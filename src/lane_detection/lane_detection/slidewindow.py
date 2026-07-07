"""슬라이딩 윈도우 차선 검출 (ROS1 slidewindow_both_lane.py 포팅).

ROS 의존성 제거: lane_side 는 노드가 set_lane_side() 로 설정한다.
로직(윈도우 좌표, polyfit 보간, 이동평균 fallback)은 원본 그대로 유지.
"""

import cv2
import numpy as np


class SlideWindow:

    def __init__(self):
        self.current_line = "DEFAULT"
        self.lane_side = "BOTH"      # "LEFT" | "RIGHT" | "BOTH"
        self.left_fit = None
        self.right_fit = None
        self.x_previous = 320

    def set_lane_side(self, side):
        if side in ("LEFT", "RIGHT"):
            self.lane_side = side
        else:
            self.lane_side = "BOTH"

    def slidewindow(self, img):
        x_location = 320
        out_img = np.dstack((img, img, img)) * 255
        height, width = img.shape[0], img.shape[1]

        window_height = 20
        nwindows = 22
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 60
        minpix = 0
        left_lane_inds = np.array([], dtype=int)
        right_lane_inds = np.array([], dtype=int)

        # 초기 탐색 윈도우 (화면 하단)
        win_h1 = 380
        win_h2 = 480

        init_margin = 60
        win_l_w_l = 145 - 120
        win_l_w_r = 145 + 100
        win_r_w_l = 495 - 100
        win_r_w_r = 495 + 120  

        circle_height = 280
        road_width = 0.545
        half_road_width = 0.5 * road_width

        left_found = False
        right_found = False

        if self.lane_side in ("LEFT", "BOTH"):
            pts_left = np.array(
                [[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]],
                np.int32,
            )
            cv2.polylines(out_img, [pts_left], False, (0, 255, 0), 1)
            good_left_inds = (
                (nonzerox >= win_l_w_l) & (nonzeroy <= win_h2)
                & (nonzeroy > win_h1) & (nonzerox <= win_l_w_r)
            ).nonzero()[0]
            if len(good_left_inds) > 0:
                left_found = True
                left_lane_inds = np.concatenate((left_lane_inds, good_left_inds))

        if self.lane_side in ("RIGHT", "BOTH"):
            pts_right = np.array(
                [[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]],
                np.int32,
            )
            cv2.polylines(out_img, [pts_right], False, (255, 0, 0), 1)
            good_right_inds = (
                (nonzerox >= win_r_w_l) & (nonzeroy <= win_h2)
                & (nonzeroy > win_h1) & (nonzerox <= win_r_w_r)
            ).nonzero()[0]
            if len(good_right_inds) > 0:
                right_found = True
                right_lane_inds = np.concatenate((right_lane_inds, good_right_inds))

        # 기준 차선 선택: 오른쪽 우선 (원본 동작), 없으면 왼쪽 폴백 (원본은 바로 MID 였음)
        # -> 코너에서 오른쪽 차선을 잠깐 놓쳐도 왼쪽 차선으로 계속 추종
        if right_found:
            line_flag = 2
        elif left_found:
            line_flag = 1
        else:
            line_flag = 3

        y_current = height - 1
        x_current = None

        if line_flag == 1 and len(left_lane_inds) > 0:
            x_current = int(np.mean(nonzerox[left_lane_inds]))
        elif line_flag == 2 and len(right_lane_inds) > 0:
            x_current = int(np.mean(nonzerox[right_lane_inds]))
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

                good_left_inds = (
                    (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                    & (nonzerox >= win_x_low) & (nonzerox < win_x_high)
                ).nonzero()[0]

                if len(good_left_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_left_inds]))
                elif len(left_lane_inds) > 0:
                    p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
                    x_current = int(np.polyval(p_left, win_y_high))

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

                good_right_inds = (
                    (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                    & (nonzerox >= win_x_low) & (nonzerox < win_x_high)
                ).nonzero()[0]

                if len(good_right_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_right_inds]))
                elif len(right_lane_inds) > 0:
                    p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
                    x_current = int(np.polyval(p_right, win_y_high))

                if circle_height - 10 <= win_y_low < circle_height + 10:
                    x_location = int(x_current - width * half_road_width)
                    cv2.circle(out_img, (x_location, circle_height), 10, (0, 0, 255), 5)

        # 원본과 동일: 검출 성공 시 x_previous 는 갱신하지 않음
        # (원본 slidewindow_both_lane.py 에서 `self.x_previous = x_location` 이
        #  주석 처리되어 있었음 — 미검출(MID) fallback 은 320 쪽으로 수렴)
        return out_img, x_location, self.current_line
