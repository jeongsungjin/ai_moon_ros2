"""슬라이딩 윈도우 차선 검출 (ROS1 slidewindow_both_lane.py 포팅 + 띠 검색 최적화).

2026-07-10 최적화: 픽셀 검색을 각 윈도우의 y 띠로 제한 (10.0ms → 5.4ms/frame).
출력 동일성 검증: 퍼징 2,100건 + 실주행 bag 13,840프레임 전 항목 비트 단위 일치.
경계 의미를 기준과 비트 단위로 동일하게 유지:
  초기 박스: y ∈ (win_h1, win_h2]  (아래 제외, 위 포함),  x ∈ [l, r] (양끝 포함)
  루프 윈도우: y ∈ [win_y_low, win_y_high),  x ∈ [low, high)
픽셀 나열 순서도 기준과 동일(행 우선) — np.mean/np.polyfit 이 비트 단위 동일해지는 근거.
"""

import cv2
import numpy as np


def _band_pixels(img, y_lo_excl_or_incl, y_hi, inclusive_high):
    """y 띠의 흰 픽셀 (절대 y 배열, x 배열) — 행 우선 순서 (기준의 nonzero 순서와 동일).

    inclusive_high=True  → y ∈ (y_lo, y_hi]   (초기 박스 의미)
    inclusive_high=False → y ∈ [y_lo, y_hi)   (루프 윈도우 의미)
    """
    h = img.shape[0]
    if inclusive_high:
        r0 = max(0, y_lo_excl_or_incl + 1)
        r1 = min(h, y_hi + 1)
    else:
        r0 = max(0, y_lo_excl_or_incl)
        r1 = min(h, y_hi)
    if r0 >= r1:
        e = np.array([], dtype=np.int64)
        return e, e
    ys_local, xs = img[r0:r1].nonzero()
    return ys_local + r0, xs


# 우측 차선 판정에 요구하는 픽셀 배수 — 좌회전 전용 트랙 좌측 편향 (2026-07-15)
RIGHT_PICK_RATIO = 1.5


def _clamped_polyval(p, y, width):
    """polyfit 외삽 폭주 클램프 (실측 -1925~+21509) — 화면 밖 좌표 차단."""
    return int(np.clip(np.polyval(p, y), 0, width - 1))


class SlideWindow:

    def __init__(self):
        self.current_line = "DEFAULT"
        self.lane_side = "BOTH"      # "LEFT" | "RIGHT" | "BOTH"
        self.left_fit = None
        self.right_fit = None
        self.x_previous = 320
        # 마지막 호출의 실제 초기 차선 검출 여부. x_location은 미검출 때도 이전값을
        # 반환하므로 값만 보고는 신뢰도를 알 수 없다.
        self.detection_valid = False
        self.initial_pixel_count = 0
        # 벡 진단용: 초기 좌/우 후보 강도와 실제 픽셀을 얻은 추적 윈도우 수.
        # 제어에는 사용하지 않고 lane_detection_node가 토픽으로 내보낸다.
        self.left_initial_pixel_count = 0
        self.right_initial_pixel_count = 0
        self.tracked_window_count = 0
        # 색으로 선의 정체성이 이미 확정된 경우 화면 좌/우 위치와 의미를 분리한다.
        # 예: 회전 중 안쪽 노란 링(LEFT)이 화면 오른쪽으로 넘어와도 RIGHT로 바꾸지 않는다.
        self.force_line_identity = None
        self.temporal_enabled = False
        self.track_centers = None
        self.temporal_margin = 45
        self.temporal_min_windows = 5

        # ---- 튜닝 파라미터 (lane 노드가 ROS 파라미터 sw_* 로 실시간 갱신) ----
        self.margin = 60             # 추적 윈도우 좌우 반폭
        self.win_h1 = 380            # 초기 탐색창 윗변 y (win_h2=480 고정)
        self.win_half = 140          # 초기 탐색창 반폭 (145/495 중심 기준)
        self.circle_height = 280     # 조향 기준점(빨간 점) y — 룩어헤드
        self.road_width = 0.51       # 차선 폭 비율 (반대편 복사 거리)
        self.nwindows = 20           # 추적 윈도우 개수
        self.minpix = 0              # 윈도우 유효 최소 픽셀 (노이즈 문턱)

    def set_lane_side(self, side):
        if side in ("LEFT", "RIGHT"):
            self.lane_side = side
        else:
            self.lane_side = "BOTH"

    def set_force_line_identity(self, side):
        self.force_line_identity = side if side in ("LEFT", "RIGHT") else None

    def reset_temporal_track(self):
        self.track_centers = None

    def set_temporal_tracking(self, enabled):
        self.temporal_enabled = bool(enabled)
        if not enabled:
            self.reset_temporal_track()

    def _track_from_prior(self, img, out_img, identity):
        """이전 프레임 윈도우 중심만 따라간다. 전역/하단 재탐색은 하지 않는다."""
        height, width = img.shape
        centers = []
        fit_y = []
        fit_x = []
        hits = 0
        x_location = self.x_previous
        for window, predicted in enumerate(self.track_centers[:self.nwindows]):
            y_hi = height - 1 - window * 20
            y_lo = y_hi - 20
            by, bx = _band_pixels(img, y_lo, y_hi, inclusive_high=False)
            m = ((bx >= predicted - self.temporal_margin)
                 & (bx < predicted + self.temporal_margin))
            good = bx[m]
            if len(good) > self.minpix:
                measured = int(np.mean(good))
                # 노란 정체성 게이트 내부에서만 각 높이의 곡선을 갱신한다. 프레임당
                # 10px 제한으로 교차선 점프를 막되 20~30도 기울어진 진입선 형상은 허용한다.
                center = int(np.clip(measured, predicted - 10, predicted + 10))
                fit_y.append(0.5 * (y_lo + y_hi))
                fit_x.append(float(measured))
                hits += 1
            else:
                center = int(predicted)
            centers.append(center)

        for window, center in enumerate(centers):
            y_hi = height - 1 - window * 20
            y_lo = y_hi - 20
            cv2.rectangle(out_img, (center-self.temporal_margin, y_lo),
                          (center+self.temporal_margin, y_hi), (0, 200, 0), 1)
        self.initial_pixel_count = hits
        self.left_initial_pixel_count = hits if identity == "LEFT" else 0
        self.right_initial_pixel_count = hits if identity == "RIGHT" else 0
        self.tracked_window_count = hits
        min_hits = min(self.temporal_min_windows, len(centers))
        fit_valid = hits >= min_hits
        if fit_valid:
            # 조향 높이 창이 비었을 때 predicted=0을 사용해 x=160이 되는 사고 방지.
            # 여러 높이에서 실제로 관측된 동일 노란 선을 1차 보간해 기준점을 계산한다.
            try:
                p = np.polyfit(np.asarray(fit_y), np.asarray(fit_x), 1)
                residual = np.abs(np.polyval(p, fit_y) - np.asarray(fit_x))
                fit_valid = float(np.median(residual)) <= self.temporal_margin
                # 고정 y=260이 관측 범위 밖이면 무리하게 외삽하거나 정지하지 않고,
                # 실제로 보이는 가장 먼 지점까지 lookahead를 당긴다. 선이 더 보이면
                # 자동으로 원래 기준 높이로 복귀한다.
                target_y = float(np.clip(
                    self.circle_height, min(fit_y), max(fit_y)))
                boundary_x = float(np.clip(
                    np.polyval(p, target_y), 0, width - 1))
                offset = width * 0.5 * self.road_width
                x_location = int(boundary_x + offset if identity == "LEFT"
                                 else boundary_x - offset)
            except (TypeError, ValueError, np.linalg.LinAlgError):
                fit_valid = False
        self.detection_valid = bool(fit_valid)
        if self.detection_valid:
            self.track_centers = centers
            self.current_line = identity
            self.x_previous = int(x_location)
            return out_img, x_location, self.current_line
        self.current_line = "MID"
        return out_img, self.x_previous, self.current_line

    def slidewindow(self, img):
        self.left_initial_pixel_count = 0
        self.right_initial_pixel_count = 0
        self.tracked_window_count = 0
        x_location = 320
        out_img = np.dstack((img, img, img)) * 255
        height, width = img.shape[0], img.shape[1]

        window_height = 20
        nwindows = self.nwindows
        margin = self.margin
        minpix = self.minpix

        # 초기 탐색 윈도우 (화면 하단) — 값은 __init__ 의 튜닝 파라미터
        win_h1 = self.win_h1
        win_h2 = 480

        win_l_w_l = 145 - self.win_half
        win_l_w_r = 145 + self.win_half
        win_r_w_l = 495 - self.win_half
        win_r_w_r = 495 + self.win_half

        circle_height = self.circle_height
        road_width = self.road_width
        half_road_width = 0.5 * road_width

        if (self.temporal_enabled and self.track_centers
                and self.force_line_identity in ("LEFT", "RIGHT")):
            return self._track_from_prior(
                img, out_img, self.force_line_identity)

        left_found = False
        right_found = False

        # 초기 박스 띠의 픽셀 (한 번만 추출, 좌/우 박스가 같은 띠를 공유)
        box_y, box_x = _band_pixels(img, win_h1, win_h2, inclusive_high=True)

        # 기준과 동일: 초기 박스 픽셀의 (y, x) 를 보관 → mean/polyfit 입력
        left_y = left_x = right_y = right_x = None

        # 초기 검출 유효 문턱: 픽셀 1개로 found 판정하면 잡음이 진짜 차선을 이김
        # (2026-07-15 리뷰: 좌커브+우측 잡음블롭 → 오차 140~299px 합성 검증)
        init_minpix = max(1, minpix)
        left_cnt = right_cnt = 0

        if self.force_line_identity is not None:
            # 이전 조향점에서 예상한 실제 경계 위치 주변을 화면 전체 하단 띠에서 찾는다.
            # LEFT 출력은 경계+반차폭, RIGHT는 경계-반차폭이므로 역산 가능하다.
            offset = width * half_road_width
            expected = self.x_previous - offset if self.force_line_identity == "LEFT" \
                else self.x_previous + offset
            radius = max(self.win_half, margin * 2)
            m = (box_x >= expected - radius) & (box_x <= expected + radius)
            forced_cnt = int(m.sum())
            if forced_cnt >= init_minpix:
                if self.force_line_identity == "LEFT":
                    left_found = True
                    left_cnt = forced_cnt
                    left_y, left_x = box_y[m], box_x[m]
                else:
                    right_found = True
                    right_cnt = forced_cnt
                    right_y, right_x = box_y[m], box_x[m]

        elif self.lane_side in ("LEFT", "BOTH"):
            pts_left = np.array(
                [[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]],
                np.int32,
            )
            cv2.polylines(out_img, [pts_left], False, (0, 255, 0), 1)
            m = (box_x >= win_l_w_l) & (box_x <= win_l_w_r)   # x 양끝 포함 (기준과 동일)
            left_cnt = int(m.sum())
            if left_cnt >= init_minpix:
                left_found = True
                left_y, left_x = box_y[m], box_x[m]

        if self.force_line_identity is None and self.lane_side in ("RIGHT", "BOTH"):
            pts_right = np.array(
                [[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]],
                np.int32,
            )
            cv2.polylines(out_img, [pts_right], False, (255, 0, 0), 1)
            m = (box_x >= win_r_w_l) & (box_x <= win_r_w_r)
            right_cnt = int(m.sum())
            if right_cnt >= init_minpix:
                right_found = True
                right_y, right_x = box_y[m], box_x[m]

        # 기준 차선 선택: 픽셀 수 기반 + 좌측 편향 (2026-07-15 좌회전 전용 트랙 대응)
        # — 좌커브 중반에 왼선이 오른쪽 초기창까지 쓸려 들어와 우측선으로 오인되는 플립 방어.
        # 우측 판정은 좌측의 1.5배 이상 픽셀일 때만 (동수 근처 애매함은 전부 좌측으로)
        if right_found and left_found:
            line_flag = 2 if right_cnt >= left_cnt * RIGHT_PICK_RATIO else 1
        elif right_found:
            line_flag = 2
        elif left_found:
            line_flag = 1
        else:
            line_flag = 3

        self.initial_pixel_count = left_cnt + right_cnt
        self.left_initial_pixel_count = left_cnt
        self.right_initial_pixel_count = right_cnt
        self.detection_valid = line_flag in (1, 2)

        y_current = height - 1
        x_current = None
        # polyfit 캐시: 입력(초기 박스 픽셀)이 루프 중 불변이라 결과가 항상 같음
        p_left = None
        p_right = None
        frame_centers = []

        if line_flag == 1 and left_x is not None and len(left_x) > 0:
            self.current_line = "LEFT"
            x_current = int(np.mean(left_x))
        elif line_flag == 2 and right_x is not None and len(right_x) > 0:
            self.current_line = "RIGHT"
            x_current = int(np.mean(right_x))
        else:
            # 차선 미검출: 이동평균으로 이전 위치 유지
            self.current_line = "MID"
            alpha = 0.9
            self.x_previous = int(alpha * self.x_previous + (1 - alpha) * x_location)
            x_location = self.x_previous
            return out_img, x_location, self.current_line

        for window in range(nwindows):
            if line_flag == 1:
                win_y_low = y_current - (window + 1) * window_height
                win_y_high = y_current - window * window_height
                win_x_low = x_current - margin
                win_x_high = x_current + margin

                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                cv2.rectangle(
                    out_img,
                    (win_x_low + int(width * road_width), win_y_low),
                    (win_x_high + int(width * road_width), win_y_high),
                    (255, 0, 0), 1,
                )

                by, bx = _band_pixels(img, win_y_low, win_y_high, inclusive_high=False)
                m = (bx >= win_x_low) & (bx < win_x_high)     # x 반개구간 (기준과 동일)
                good_x = bx[m]

                if len(good_x) > minpix:
                    self.tracked_window_count += 1
                    x_current = int(np.mean(good_x))
                elif len(left_x) > 0:
                    if p_left is None:
                        p_left = np.polyfit(left_y, left_x, 2)
                    x_current = _clamped_polyval(p_left, win_y_high, width)
                frame_centers.append(int(x_current))

                if circle_height - 10 <= win_y_low < circle_height + 10:
                    x_location = int(x_current + width * half_road_width)
                    cv2.circle(out_img, (x_location, circle_height), 10, (0, 0, 255), 5)

            elif line_flag == 2:
                win_y_low = y_current - (window + 1) * window_height
                win_y_high = y_current - window * window_height
                win_x_low = x_current - margin
                win_x_high = x_current + margin

                cv2.rectangle(
                    out_img,
                    (win_x_low - int(width * road_width), win_y_low),
                    (win_x_high - int(width * road_width), win_y_high),
                    (0, 255, 0), 1,
                )
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)

                by, bx = _band_pixels(img, win_y_low, win_y_high, inclusive_high=False)
                m = (bx >= win_x_low) & (bx < win_x_high)
                good_x = bx[m]

                if len(good_x) > minpix:
                    self.tracked_window_count += 1
                    x_current = int(np.mean(good_x))
                elif len(right_x) > 0:
                    if p_right is None:
                        p_right = np.polyfit(right_y, right_x, 2)
                    x_current = _clamped_polyval(p_right, win_y_high, width)
                frame_centers.append(int(x_current))

                if circle_height - 10 <= win_y_low < circle_height + 10:
                    x_location = int(x_current - width * half_road_width)
                    cv2.circle(out_img, (x_location, circle_height), 10, (0, 0, 255), 5)

        # 검출 성공 시 x_previous 갱신 (2026-07-15): 원본은 갱신하지 않아 미검출 폴백이
        # 항상 320(중앙) 으로 떨어졌음 — 이제 미검출 시 "마지막으로 본 위치"를 유지
        self.x_previous = int(x_location)
        if self.temporal_enabled and frame_centers:
            self.track_centers = frame_centers
        return out_img, x_location, self.current_line

