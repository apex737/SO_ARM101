"""노트북(GPU PC) 쪽 실행 코드: 위 카메라 블록 검출, 화면 → 로봇 좌표 보정, 팔 기구학.

- vision.py      : 사진에서 HSV 색으로 블록을 찾고 중심 픽셀·각도를 구한다 (numpy, OpenCV만 사용)
- calib.py       : 보정점 쌍으로 화면 픽셀 → 로봇 좌표(mm) 변환을 구하고 적용한다
- kinematics.py  : SO-101 URDF로 관절값 ↔ 그리퍼 끝 위치를 계산한다 (placo 필요 → conda lerobot 환경)
- camera.py      : USB 웹캠 열기
- view_blocks.py : 실시간 검출 화면 (python -m kitbot_exec.view_blocks)
"""
