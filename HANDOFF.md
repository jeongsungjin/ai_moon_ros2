# ai_moon_ros2 인수인계 (2026-07-15, SEA:ME 해커톤 당일)

> Fable 5 세션 종료로 Codex에 인계. 대회 당일 오후 경기. **코드는 전부 미커밋 상태** (e85ff0f "배설" 이후 이틀치).
> 읽는 순서: §1 현재상태 → §2 남은 할 일 → §3 규칙 → 나머지는 필요할 때 참조.

---

## 1. 지금 어디까지 왔나

### 미션 체인 전 구간 실측 통과 (마지막 벡 `bags/run_0715_162248`, 91초)

| t | 이벤트 | 판정 |
|---|---|---|
| 7.17s | GREEN GLOW 즉발 출발 (startup_ignore 6.5 만료 직후) | ✅ |
| 12.70s | 회전교차로 ARMED | ✅ |
| 14.31s | 노랑 트리거 → 0.8s 딜레이 → LOOP + 부스트 2.5s | ✅ |
| 14.3~25.8s | 링 주행. 조향 부호반전 0~1회/초, x_location ±40 | ✅ |
| 25.80s | **EXIT — 적분 2.709 도달 (11.5s)**. 타임아웃(15s) 아님 | ✅ |
| 28.85s | DONE 래치 | ✅ |
| 59.06~61.36s | 아루코 장애물 정지 → 재출발 (thr 0.20→0.00→0.20) | ✅ |
| 19.8/20.5s | red 검출됐으나 무시됨 (min_drive 60s 게이트) | ✅ 의도대로 |
| 33~90s | 일반 주행 x=286 ±62, **프레임 점프>40px 0회/1425f** | ✅ |

**직전 수정 2건, 이 벡에서 효과 확인됨:**
- `entry_boost_sec` 1.5 → **2.5** — 진입 후 휘청임 해소 (핸드오프가 커브 안정 구간으로 이동)
- `steer_integral_target` 4.5 → **2.7** — 앞 벡(`run_0715_161501`) 실측 기반. **이번 벡에서 적분 2.709로 정상 EXIT 발동 확인** = 이 값 확정

### 아직 못 본 것 (유일한 미검증)
**결승 red 정지 end-to-end.** 5중 게이트(60s + not in_red_zone + passed_red_zone + 크기/종횡비/발광 + red_stable 1)를 전부 통과한 실제 정지를 아직 못 봄. 위 벡도 91초에서 끊겨 `lap: [0]`.
- 주의: 이 벡에서 `passed_red_zone` 활성화 로그가 안 보임 → 아루코 구간 통과 후 `/is_red` True→False 전이(67.5s OUT)가 DRIVING 중 발생했으므로 열렸어야 함. **다음 벡에서 "빨간 구간 통과 — 결승 red 감시 활성화" 로그 확인 필요.** 안 열려도 90초 fallback이 백업.

---

## 2. 남은 할 일 (우선순위)

1. **커밋** ← 최우선. 이틀치 튜닝이 전부 워킹트리에만 있음. 사용자가 "한 바퀴 잘 돌면 커밋"에 동의함. **단, 사용자 지시 없이는 절대 커밋 금지 (§3)**
2. 결승 red 정지 end-to-end 1회 확인 (60초 넘게 주행 → 결승등 red → 정지)
3. 경기 직전 체크리스트:
   - fresh 배터리 + `motor_test` 30초 (문턱 0.18·트림 -0.007 재확인 — 전압 따라 변함)
   - `mfps` 확인
   - `use_web_viewer:=false` + `mdog`
   - E-stop 대기
4. (경기 후) `dead-code-inventory` 일괄 정리

---

## 3. 절대 규칙 (사용자 명시)

- **git 커밋은 사용자가 시킬 때만.** 제안까지만. `tuning` 브랜치 전용, **main 절대 금지**
- **dead code는 "없애자" 신호 전까지 제거 금지.** 목록만 유지 (`~/.claude/.../memory/dead-code-inventory.md`)
- **파라미터/로직 변경 후 반드시 `diff src/... install/...`로 install 반영 확인** (아래 §6 함정)
- 검증된 일반주행 제어값(gain/lane_center_x/circle_height)은 링 문제 때문에 흔들지 말 것

---

## 4. 확정 파라미터 (`src/car_planner/config/params.yaml`)

전부 실측으로 확정된 값. **근거 없이 바꾸지 말 것** — 각 줄에 주석으로 근거가 달려 있음.

```
# 제어 (Plan D = 최적 폐루프 상태)
speed_safe 0.22 | steering_gain 0.0030 | steer_trim -0.007 | lane_center_x 340.0
sw_circle_height 260 | sw_margin 75 | sw_nwindows 14 | sw_minpix 15
curve_slow_gain 0.6 | curve_slow_floor 0.91 | throttle_slew_up 0.012

# 인지 (그레이스케일 adaptive — 대회장 조명 편차로 HSV 전면 폐기)
adaptive_block 113 | adaptive_c -60 | bev_edge_mask 60 | morph_open 0
min_blob_area 800 | min_blob_height 80
x_ema_alpha 0.5 | x_max_step 60 | x_hold_frames 0(기각됨)

# 회전교차로
yellow_chroma_thresh 60 | yellow_roi_y0 400 | yellow_arm_threshold 800
entry_trigger_delay_sec 0.8 | entry_boost_sec 2.5 | entry_angle -120 | entry_speed 0.2
entry_min_sec 5.5 | entry_max_sec 9.0 | entry_lane_side_sec 15
loop_sec 15 | steer_integral_target 2.7 | exit_angle -100 | exit_speed 0.2 | exit_duration 2.0

# 신호등
green_stable_frames 1 | red_stable_frames 1 | green_conf_threshold 0.15
glow_min_px 22 | glow_roi_y0 200 | startup_ignore_sec 6.5
min_drive_time_sec 60 | red_zone_fallback_sec 90 | lap_count 1
imgsz 640 | conf 0.4 | infer_hz 2.5 | min_box_height_px 12 | max_box_height_px 70
use_red_gate true
```

**⚠️ `steer_integral_target 2.7`은 링 속도 0.2 기준 실측.** speed 계열을 건드리면 이 값도 재실측 대상.

**⚠️ `sw_circle_height`(ld) ↔ `sw_nwindows` 결합:** 윈도우 계단이 ld에 닿아야 x_location이 갱신됨. `(460-ld)/20+1 ≤ nwin`. 이거 어겨서 직진 불능 사고 난 적 있음.

---

## 5. 대회장 확정 사실 (7/14~15 현장)

- **1랩 규칙.** 마지막 신호등 red 정지 = 즉시 FINISH
- **노란선은 회전교차로에만 존재** → 노랑 카운트를 링 진입 트리거로 쓸 수 있는 근거
- 조명 편차(글레어↔암부)로 **HSV 절대값 튜닝 불가 → 그레이스케일 adaptive 전면 전환**. `/yellow_pixels`는 이제 BGR 산술 노랑(min(R,G)-B>60), ROI y≥400
- 신호등은 **바닥에서 30cm 크기** (박스 20~40px) → max_box 70으로 초록옷/대형스크린 오발 차단
- 좌회전 전용 트랙 → `lane_center_x 340`(상시 좌측 붙기) + SlideWindow 좌측 편향(RIGHT_PICK_RATIO 1.5) "꼼수" 적용됨

---

## 6. 이 환경의 함정 (반복해서 당함)

1. **colcon copy-skip**: 타임스탬프 이상 시 파일을 조용히 복사 안 함. **빌드 후 매번 `diff src/X install/X`.** 막히면 `rm -rf build/X install/X` 클린 빌드
2. **셸 cwd가 세션마다 `/home/topst`로 리셋**. 명령마다 `cd /home/topst/ai_moon_ros2 &&` 붙일 것
3. 강제종료(force-kill)가 install 메타데이터를 깨뜨린 적 있음
4. `setup.cfg`에 오타 흘러들어가서 빌드 깨진 적 있음 (`git checkout`으로 복구)

---

## 7. 도구

```
mauto   # race.launch.py 풀스택 (경기용)
mbag    # 로스벡 기록 — 이번에 19토픽으로 확장함 (§8)
mfps    # fps 확인 (camera 23.7 / lane 25.1 @ YOLO 640 — 정상)
mfix    # 방전 후 복구
mplay   # bag_player.py (:8084)
mpad    # gamepad_test.py
mhsv    # web_tuner.py
mdog    # 워치독
```

**`mbag` 확장 (오늘 추가)** — 이제 bag만으로 전체 분석 가능, 터미널 로그 놓쳐도 됨:
```
/camera/image/compressed /lane_detection/image/debug /lane_x_location /yellow_pixels
/motor_lane /mode /control /e_stop /traffic_sign
/mission/roundabout_state /mission/traffic_state /mission/dynamic_state /mission/lap
/roundabout/steer_integral /roundabout/loop_elapsed
/yolo/green /yolo/red /is_red /aruco/visible
```
- `/roundabout/steer_integral`, `/loop_elapsed` — 링 적분 실측용 (이거 없어서 며칠 헤맴)
- `/lane_detection/image/debug` — 라이브 슬라이딩윈도우 화면 (오프라인 재구성 불필요해짐), ~8MB/분

### bag 분석 방법 (ROS2 설치 없이 sqlite 직독 — 빠름)
```python
import sqlite3, struct
db = sqlite3.connect('bags/run_XXXX/run_XXXX_0.db3')
tid = {r[1]: r[0] for r in db.execute("SELECT id,name FROM topics")}
t0 = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
# std_msgs: Float32 = struct.unpack_from('<f', d, 4)[0]; Int32 = '<i'; Bool = d[4]!=0
# String: n=struct.unpack_from('<I',d,4)[0]; d[8:8+n-1].decode()
# control_msgs/Control (header 있음!): off=4+8; n=unpack('<I',d,off); off+=4+n; off=(off+3)&~3
#                                      steering,throttle = struct.unpack_from('<ff', d, off)
```
CompressedImage 디코딩은 header(8B) → frame_id string → format string → data len 순으로 오프셋 정렬 필요.

---

## 8. 아키텍처 요약

**제어권 중재**: 각 미션 노드가 `/motor_*` (drive_msgs/DriveCommand, `flag=True`=제어권 요청)를 발행 → car_planner가 우선순위로 중재 → `/control` → control_node → PCA9685.
- `traffic_light_mission` → `/motor_sign` (WAIT_GREEN/FINISH에서 flag=True 정지)
- `roundabout_mission` → `/motor_roundabout` (LOOP 부스트/EXIT에서 flag=True)
- `dynamic_obs_mission` → `/motor_dynamic` (아루코 보이면 flag=True 정지)
- `lane_detection` → `/motor_lane` (평시 주행)

**lane 방어 체인** (이 순서 중요, 다 실측으로 필요성 입증됨):
runaway guard(x∈[0,640] 아니면 직전값 유지) → rate limiter(60px/frame) → EMA(alpha 0.5) → P 제어(gain 0.0030, `error = x_location - lane_center_x`)

**회전교차로 FSM**:
`IDLE → (entry_min 5.5s) ARMED → (하단밴드 노랑>800) 트리거 → (0.8s 딜레이) LOOP` → LOOP 진입 시 부스트 2.5s(`lane_angle + entry_angle(-120)`, speed 0.2) → `적분 2.7 도달 or loop_sec 15 타임아웃` → `EXIT`(RIGHT + exit_angle -100, 2.0s) → `DONE` 래치

**신호등 5중 게이트** (결승 red 오발 방지 — 사용자가 빨간바닥/빨간옷 반복 우려):
`min_drive 60s` + `not in_red_zone` + `passed_red_zone`(DRIVING 중에만 래치, 90s fallback) + `크기/종횡비/발광 검증` + `red_stable 1`

**GREEN GLOW 패스트패스**: YOLO가 이 램프에 약해서(실측 conf 0.25~0.29) 카메라 프레임에서 초록 발광 픽셀 직독(8Hz, ROI y≥200, ≥22px) → WAIT_GREEN에서만 작동. 사용자 요구는 "즉발 출발", 한 틱이면 충분.

---

## 9. 해결된 사고 이력 (재발 시 참조 — 같은 함정 다시 파지 말 것)

| 증상 | 진짜 원인 | 조치 |
|---|---|---|
| 링 도중 +20.4s에 영구정지 | 출발 신호등 red를 결승으로 오판 | min_drive 20→60s + 순서 게이트 |
| 초록불 미검출 | `predict()`가 global conf 0.4에서 컬링 → green 임계 적용 전에 죽음 | `pred_conf = min(conf, green_conf)` |
| 없는 초록불에 출발 | 초록 옷 오인 | max_box 70 + 발광 검증 |
| 기동 직후 의도치 않은 2초 주행 | planner가 traffic 미션보다 먼저 뜸 | control_node를 TimerAction(5s)로 지연 기동 |
| ↑의 부작용: 정지 상태로 진입창 소진 | control 뜨기 전에 green 수락 | `startup_ignore_sec 6.5` |
| **주행 중 모터 급사 (3회)** | **명령은 흐르는데 바퀴 정지 = ESC 저전압 컷.** 돌진 전류가 배터리 순간 강하 (프레임차 모션 재구성으로 진단) | `throttle_slew_up 0.012` (증가만 램프, 감속은 즉시). **이후 재발 없음** |
| 커브에서 스톨 | 평지 구동 문턱 0.18이나 조향 꺾이면 저항 증가로 실효 문턱 상승 | `curve_slow_floor 0.91` (하한 0.200 보장) |
| 회전교차로 조기 진입 (반복) | 노랑 카운트가 "도착"이 아니라 "보임"을 재고 있었음 | ROI를 y≥400(차 바로 앞 80px)로 제한 + 0.8s 트리거 딜레이 |
| 진입 후 휘청휘청 | 부스트→차선 핸드오프가 커브 한복판에 떨어짐 | `entry_boost_sec` 1.5→2.5 (해결 확인) |
| 커브 정상 변화에 휘청 | `x_hold_frames` 점프 홀드가 참 변화도 막음 | **사용자가 기각 → 0.** 파라미터는 남겨둠(dead-code 목록 A) |
| sw_margin 60 실험 | 좁은 박스가 선을 상시 놓침/재획득 → 지터 2~3배 | **기각**, 75 유지 |

---

## 10. 사용자(문영) 협업 스타일

- **실증 요구**: "아마도"를 싫어함. 벡/실측 데이터로 말할 것. 근거 없는 파라미터 제안 = 거부됨
- **통제권 유지**: 제안까지만. 커밋·삭제 같은 되돌리기 어려운 건 반드시 지시 대기
- **안전 우선**, 시각 도구 선호 (그리드 이미지/디버그 프레임 좋아함)
- 제안할 땐 **인지 영향 + CPU 비용 + 검증법**을 같이 줄 것
- 한국어로 대화

---

## 11. 알려진 열린 이슈

- **링 27초 부근 X자 교차 외란**(앞 벡 `run_0715_161501`): 진입 길목에서 선이 교차하며 x가 402±111로 튐. 방어 체인이 복구시켜 주행엔 지장 없었음. 근본 해결 안 됨 — 경기용으론 수용 가능 판단
- 링 휘청임의 잔여분: 작은 반경 + P제어/룩어헤드 기하의 본질적 한계. 더 잡으려면 LOOP 전용 룩어헤드(`sw_circle_height` 290으로 param-set 후 DONE에서 복원, ~10줄) 또는 `x_ema_alpha` 0.5→0.4. **둘 다 미적용 — 현 상태로 충분하다고 판단**
- YOLO가 자기 taskset 코어 100% 점유 (허용 범위)
