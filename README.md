# AI_moon ROS2 (JetRacer / Camera-Only)

2024 스케일카 자율주행 (ROS1 Noetic, `ai_moon/`) 코드를 **ROS2 Humble** 로 마이그레이션한
워크스페이스. 하드웨어는 JetRacer(카메라 온리), 구동 규약은
[TOPST D-Racer-Kit](https://github.com/topst-development/D-Racer-Kit) 을 따른다.

## 아키텍처 (ROS1 구조 유지)

```
[camera]  /camera/image/compressed (CompressedImage, jpeg)
    │
    ├── [lane_detection]  HSV → 원근변환 → 슬라이딩윈도우 → /motor_lane (DriveCommand)
    ├── [cv_detect]       parking_sign → /is_blue, rubbercone → /is_orange
    │
[car_planner/main_planner]  /motor_* flag 우선순위 중재 → MODE 결정
    │                        SIGN > RABACON > STATIC > DYNAMIC > ROUNDABOUT
    │                        > TUNNEL > PARKING > LANE
    ▼
/control (control_msgs/Control: steering/throttle, -1~+1)
    │
[control]  PCA9685 → 조향 서보(ch0) + ESC(ch1)
```

### ROS1 → ROS2 변경 요약

| 항목 | ROS1 (기존) | ROS2 (현재) |
|------|-------------|-------------|
| 프레임워크 | rospy + catkin | rclpy + colcon (ament_python) |
| 카메라 입력 | `/usb_cam/image_raw` (Image + cv_bridge) | `/camera/image/compressed` (CompressedImage + imdecode) |
| 미션 명령 | `Drive_command` (패키지별 중복 정의) | `drive_msgs/DriveCommand` (단일 정의) |
| 최종 출력 | `AckermannDriveStamped` → VESC | `control_msgs/Control` → PCA9685 |
| LiDAR/IMU | `/raw_obstacles`, `/heading` 구독 | **제거** (카메라 온리) |
| HSV 튜닝 | OpenCV 트랙바 | ROS2 파라미터 (`config/params.yaml`) |
| 루프 | `while + rospy.Rate(30)` | `create_timer(1/30)` |

기존 LiDAR 기반 미션(rabacon_drive, detect_clust_obs)은 제거됨.
새 미션 노드는 `drive_msgs/DriveCommand` 를 `/motor_<mission>` 으로 발행(flag=True 로
제어권 요청)하면 플래너가 자동으로 중재한다 — `main_planner_node.py` 의
`MISSION_PRIORITY` 에 토픽이 이미 등록되어 있음.

## 패키지

| 패키지 | 타입 | 내용 |
|--------|------|------|
| `drive_msgs` | ament_cmake | `DriveCommand.msg` (speed, angle, flag) |
| `control_msgs` | ament_cmake | `Control.msg` (header, steering, throttle) — D-Racer-Kit 호환 |
| `camera` | ament_python | USB(V4L2)/CSI(GStreamer) → CompressedImage 발행 |
| `lane_detection` | ament_python | HSV + 슬라이딩윈도우 차선인식, 빨강 감속·흰색 정지 미션 포함 |
| `cv_detect` | ament_python | 주차표지판(파랑), 라바콘(주황) 픽셀 검출 |
| `car_planner` | ament_python | 미션 중재 플래너 + launch + `config/params.yaml` |
| `control` | ament_python | `/control` → PCA9685 서보/ESC 구동, e-stop·타임아웃 안전장치 |

> ⚠️ `control_msgs` 는 ros-controls 의 공식 `control_msgs` 와 이름이 겹친다
> (D-Racer-Kit 규약을 따른 것). `ros-humble-control-msgs` 가 설치된 환경에서는
> 이 워크스페이스의 오버레이가 우선되지만, MoveIt 등과 혼용하지 말 것.

## 빌드

```bash
cd ai_moon_ros2
rosdep install --from-paths src --ignore-src -r -y   # 의존성 설치
colcon build --symlink-install
source install/setup.bash
```

## 실행

```bash
# 전체 스택 (실차)
ros2 launch car_planner auto_driving.launch.py

# 차선 주행만
ros2 launch car_planner lane_drive.launch.py

# 개발 PC (하드웨어 없이) — control 노드 제외
ros2 launch car_planner auto_driving.launch.py use_control:=false

# 비상 정지
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```

디버그: `/lane_detection/image/debug` (슬라이딩윈도우 시각화, jpeg) 를
`rqt_image_view` 나 D-Racer-Kit monitor 대시보드로 확인.

## 튜닝 포인트 (`src/car_planner/config/params.yaml`)

- **HSV 범위**: 대회장 조명에 맞게 `hsv_*` 값 조정 (기존 트랙바 대신 파라미터)
- **`steering_gain`**: 픽셀 오차(±320) → 조향 percent(±1). 기본 0.003
  - 조향 방향이 반대면 `invert_steering: true`
- **`throttle_gain` / `max_throttle`**: 속도값(0.2~0.5) → 스로틀 percent 변환/상한
- **`i2c_bus`**: JetRacer(Jetson)=1, TOPST D3-G=3
- **`steer_trim`**: 직진 보정
- 서보/ESC 펄스 범위: `servo_*_us`, `esc_*_us` (control_node 파라미터)

## 대회 킷과 함께 쓰는 경우

D-Racer-Kit 의 `camera`/`control`/`monitor` 노드를 그대로 쓴다면 이 워크스페이스의
`camera`, `control`, `control_msgs` 패키지는 빌드에서 제외해도 된다
(`touch src/camera/COLCON_IGNORE` 등). 토픽/메시지 규약이 동일해서
`lane_detection` + `car_planner` 만으로 바로 연동된다.
