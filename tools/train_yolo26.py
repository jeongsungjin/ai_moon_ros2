#!/usr/bin/env python3
"""YOLO26n 신호등/표지판 학습 파이프라인.

데이터셋: perception.yolo26/ (Roboflow, 4클래스: green/left/red/right)
  - 라벨이 폴리곤(세그먼테이션) 포맷이지만 ultralytics 가 detect 학습 시
    자동으로 bbox 로 변환하므로 별도 전처리 불필요.

사용 (GPU PC 또는 Colab):
  pip install "ultralytics>=8.3.200"        # YOLO26 지원 버전
  python3 tools/train_yolo26.py                       # 기본 학습
  python3 tools/train_yolo26.py --epochs 150 --batch 32
  python3 tools/train_yolo26.py --eval-only --weights runs/detect/traffic_yolo26n/weights/best.pt

학습 완료 후:
  runs/detect/traffic_yolo26n/weights/best.pt
  → ai_moon_ros2/models/yolo26n_traffic.pt 로 복사하면 yolo_detect 노드가 사용
"""

import argparse
from pathlib import Path

WS_ROOT = Path(__file__).resolve().parents[1]          # ai_moon_ros2/
DEFAULT_DATA = WS_ROOT / 'perception.yolo26' / 'data.yaml'
DEFAULT_MODEL_OUT = WS_ROOT / 'models' / 'yolo26n_traffic.pt'


def parse_args():
    p = argparse.ArgumentParser(description='Train YOLO26n for traffic light/sign detection')
    p.add_argument('--data', type=str, default=str(DEFAULT_DATA), help='data.yaml 경로')
    p.add_argument('--model', type=str, default='yolo26n.pt', help='사전학습 가중치 (yolo26n.pt 자동 다운로드)')
    p.add_argument('--epochs', type=int, default=100)
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--batch', type=int, default=16, help='GPU 메모리에 맞게 조정 (-1: 자동)')
    p.add_argument('--device', type=str, default=None, help="예: '0', 'cpu', 'mps' (기본: 자동)")
    p.add_argument('--name', type=str, default='traffic_yolo26n', help='runs/detect/<name>')
    p.add_argument('--patience', type=int, default=30, help='early stopping patience')
    p.add_argument('--export-onnx', action='store_true', help='학습 후 ONNX 도 export (보드 가속용)')
    p.add_argument('--eval-only', action='store_true', help='학습 없이 test 셋 평가만')
    p.add_argument('--weights', type=str, default=None, help='--eval-only 시 평가할 가중치')
    p.add_argument('--copy-to-models', action='store_true', default=True,
                   help=f'학습 후 best.pt 를 {DEFAULT_MODEL_OUT} 로 복사 (기본 on)')
    return p.parse_args()


def main():
    args = parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            'ultralytics 가 설치되어 있지 않습니다:\n'
            '  pip install "ultralytics>=8.3.200"'
        )

    data_yaml = Path(args.data)
    if not data_yaml.exists():
        raise SystemExit(f'data.yaml 을 찾을 수 없음: {data_yaml}')

    # ---------------- 평가 전용 모드 ----------------
    if args.eval_only:
        weights = args.weights or str(DEFAULT_MODEL_OUT)
        if not Path(weights).exists():
            raise SystemExit(f'가중치 파일 없음: {weights}')
        model = YOLO(weights)
        print(f'\n=== test 셋 평가: {weights} ===')
        metrics = model.val(data=str(data_yaml), split='test', imgsz=args.imgsz,
                            device=args.device)
        print(f'mAP50: {metrics.box.map50:.4f} | mAP50-95: {metrics.box.map:.4f}')
        for i, name in metrics.names.items():
            print(f'  {name}: mAP50-95={metrics.box.maps[i]:.4f}')
        return

    # ---------------- 학습 ----------------
    model = YOLO(args.model)
    results = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        name=args.name,
        patience=args.patience,
        # 주행 카메라 특성상 좌우 반전 금지: left/right 표지판 라벨이 뒤집힘!
        fliplr=0.0,
        plots=True,
    )

    best = Path(results.save_dir) / 'weights' / 'best.pt'
    print(f'\n=== 학습 완료: {best} ===')

    # test 셋 최종 평가
    model = YOLO(str(best))
    metrics = model.val(data=str(data_yaml), split='test', imgsz=args.imgsz, device=args.device)
    print(f'test mAP50: {metrics.box.map50:.4f} | mAP50-95: {metrics.box.map:.4f}')

    # 배포 위치로 복사
    if args.copy_to_models:
        DEFAULT_MODEL_OUT.parent.mkdir(parents=True, exist_ok=True)
        import shutil
        shutil.copy2(best, DEFAULT_MODEL_OUT)
        print(f'배포용 복사 완료: {DEFAULT_MODEL_OUT}')
        print('→ 보드에서: ros2 launch yolo_detect yolo_detect.launch.py')

    if args.export_onnx:
        onnx_path = model.export(format='onnx', imgsz=args.imgsz, simplify=True)
        print(f'ONNX export: {onnx_path}')


if __name__ == '__main__':
    main()
