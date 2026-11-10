# SO-ARM101 캘리브레이션 파일 복사본

LeRobot은 `~/.cache/huggingface/lerobot/calibration/`의 파일만 읽는다. 여기는 git으로 이력을 남기기 위한 복사본이다.

- 팔로워암: `robots/so_follower/my_follower_arm.json` (`--robot.id=my_follower_arm`)
- 리더암: `teleoperators/so_leader/my_leader_arm.json` (`--teleop.id=my_leader_arm`)
- 다시 캘리브레이션하면 여기로 다시 복사하고 커밋한다. 수집한 학습 데이터는 그때의 캘리브레이션 기준이다.

되돌리기(저장소 → LeRobot 위치):
```bash
cp -r project/src/calibration/robots project/src/calibration/teleoperators ~/.cache/huggingface/lerobot/calibration/
```
