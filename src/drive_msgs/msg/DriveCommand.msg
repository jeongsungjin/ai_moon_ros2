# 미션 노드 -> main_planner 로 전달되는 주행 명령
# (ROS1 Drive_command.msg 를 ROS2 네이밍 규칙에 맞게 DriveCommand 로 변경)
float32 speed   # 목표 속도
float32 angle   # 조향 오차/각 (차선: x_location - 320 픽셀 오차)
bool flag       # true = 해당 미션이 제어권을 요청함
