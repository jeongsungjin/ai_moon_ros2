# models/

학습된 YOLO26 가중치 배치 폴더 (`.pt` 파일은 git 미추적).

- `yolo26n_traffic.pt` — 신호등/표지판 4클래스 (green/left/red/right)
  - 학습: `python3 tools/train_yolo26.py` (GPU PC/Colab)
  - 학습 스크립트가 자동으로 이 위치로 best.pt 를 복사함
  - 보드로 옮길 때: `scp models/yolo26n_traffic.pt <board>:~/ai_moon_ros2/models/`
