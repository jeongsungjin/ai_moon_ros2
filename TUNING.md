# 🔧 현장 튜닝 가이드 (tuning 브랜치)

> **이 브랜치는 현장 튜닝 전용입니다.**
> 튜닝값은 전부 `src/car_planner/config/params.yaml` 에 있습니다.
> **로직 코드(`*.py`)는 수정하지 마세요** — 로직 변경이 필요하면 로직 담당(성진)에게 연락.

## 0. 기본 워크플로우

```bash
cd ~/ai_moon_ros2
git pull origin tuning              # 시작 전 최신값 받기

# ... 튜닝 작업 ...

# 값이 확정되면 저장 (params.yaml 만 커밋!)
git add src/car_planner/config/params.yaml
git commit -m "tune: 대회장 오전 조명 HSV + steering_gain 0.004"
git push origin tuning
```

커밋 메시지에 **언제/어떤 조건**(오전/오후, 조명, 트랙 상태)이었는지 적어주세요.
로직 담당이 나중에 `main` 에 반영합니다.

## 1. 실행 명령어

```bash
source /opt/ros/humble/setup.bash && source ~/ai_moon_ros2/install/setup.bash

ros2 launch car_planner lane_drive.launch.py                      # 차선 주행
ros2 launch car_planner lane_drive.launch.py use_control:=false   # 모터 없이 인지만
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"    # ⛔ 비상 정지
```

> `params.yaml` 수정 후에는 launch 재시작만 하면 됩니다 (재빌드 불필요, symlink 설치).
> 단, **처음 clone 했다면** `colcon build --symlink-install` 1회 필요.

## 2. 튜닝 순서 (권장)

### STEP 1 — 카메라 확인
```bash
ros2 topic hz /camera/image/compressed     # 30Hz 근처인지
```
- 화면이 뒤집혀 있으면: `camera_node: flip_180: true`
- CSI 카메라면: `camera_type: csi`

### STEP 2 — 차선 인식 (HSV)
모니터 연결 시 (제일 편함):
```bash
# params.yaml 에서 lane_detection_node: show_gui: true 로 바꾸고 launch
# → 트랙바 창에서 실시간 조정, imshow 로 마스크 확인
```
노트북 원격 (같은 WiFi):
```bash
ros2 run rqt_image_view rqt_image_view     # /lane_detection/image/debug 선택
ros2 param set /lane_detection_node hsv_yellow_lower "[10, 100, 120]"   # 실시간 반영
```
- **판정 기준**: 디버그 영상에서 슬라이딩윈도우(초록/파랑 박스)가 차선을 따라가고,
  빨간 원(차선 중심 추정)이 도로 중앙에 있으면 OK
- ⚠️ **트랙바/param set 으로 찾은 값은 노드 끄면 사라짐** → 반드시 params.yaml 에 옮겨 적기

### STEP 3 — 조향 방향/게인 (차 들어올리고!)
```bash
ros2 param set /main_planner invert_steering true    # 조향이 반대로 꺾이면
ros2 param set /main_planner steering_gain 0.004     # 코너 못 따라감 → 올리기
                                                     # 좌우로 출렁임(지그재그) → 내리기
ros2 param set /control_node steer_trim 0.05         # 직진인데 한쪽으로 쏠림
```

### STEP 4 — 속도
처음엔 `control_node: max_throttle: 0.3` 으로 낮춰서 시작 → 안정되면 올리기.
- `lane_detection_node: speed_safe` : 기본 주행 속도
- `main_planner: throttle_gain` : 전체 속도 스케일

## 3. 주요 파라미터 표 (params.yaml)

| 증상 | 파라미터 | 방향 |
|------|----------|------|
| 차선을 마스크가 못 잡음 | `hsv_yellow_*`, `hsv_white_*` | show_gui 트랙바로 탐색 |
| 조향 반대 | `main_planner: invert_steering` | true |
| 코너에서 못 꺾음 | `main_planner: steering_gain` | ↑ (0.003 → 0.004~0.006) |
| 직선에서 지그재그 | `main_planner: steering_gain` | ↓ (0.003 → 0.002) |
| 직진인데 쏠림 | `control_node: steer_trim` | 쏠리는 반대쪽으로 ±0.05 씩 |
| 너무 빠름/느림 | `control_node: max_throttle` | 0.3 에서 시작해 조정 |
| 빨간 구간 감속 오작동 | `red_pixel_threshold` | 오탐이면 ↑ |
| 횡단보도 정지 오작동 | `white_pixel_threshold` | 오탐이면 ↑ |
| 모터가 아예 안 돎 | `control_node: i2c_bus` | JetRacer=1, D3-G=3 확인 |
| 서보 각도 범위 이상 | `servo_center_us`, `servo_span_us` | 중립 1500 기준 조정 |

## 4. 상태 확인 명령어

```bash
ros2 topic echo /lane_x_location    # 320 = 정중앙 (차선 중심 추정치)
ros2 topic echo /motor_lane         # 차선 노드 출력 (speed/angle/flag)
ros2 topic echo /control            # 최종 조향/스로틀 (-1~+1)
ros2 topic echo /mode               # 현재 모드 (LANE 이어야 정상)
ros2 param dump /lane_detection_node   # 현재 적용된 파라미터 전체 확인
```

## 5. 문제 생겼을 때

- **빌드 에러 / 노드 크래시** → 터미널 로그 전체를 복사해서 로직 담당에게 전송
- **smbus2 없다고 나옴** → `pip3 install smbus2`
- **롤백**: `git checkout src/car_planner/config/params.yaml` (마지막 커밋값으로 복구)
