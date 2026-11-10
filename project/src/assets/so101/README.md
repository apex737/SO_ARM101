# SO-101 팔 모델 (URDF + 메시)

- 출처: https://github.com/TheRobotStudio/SO-ARM100 의 `Simulation/SO101/` (2026-10-06 커밋 a758567, 2026-11-10 받음)
- 받은 파일: `so101_new_calib.urdf`와 이 파일이 쓰는 메시(STL) 13개만. 라이선스는 원 저장소를 따른다.
- `new_calib` = 각 관절의 0°가 가동 범위 가운데인 모델. LeRobot의 새 캘리브레이션 방식과 같은 기준이다.
- 메시가 없으면 계산 라이브러리(placo)가 모델을 읽지 못해서 함께 둔다(약 16MB).
- 사용: `project/src/kitbot_exec/kinematics.py`
