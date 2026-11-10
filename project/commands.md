# 자주 쓰는 명령 (SO-ARM101 · Ubuntu 작업 PC)

LeRobot 0.6.0 기준으로 이 PC의 `--help` 출력과 `~/lerobot/docs`에서 인자를 확인했다(2026-11-10).
포트·카메라 값은 2026-11-10에 이 PC에서 확인한 값이다. 재부팅하거나 USB를 다시 꽂으면 바뀔 수 있으니 2번으로 다시 확인한다.

## 0. 어떤 가상환경을 쓰나

| 할 일 | 가상환경 | 켜기 |
|---|---|---|
| 팔 캘리브레이션·원격 조종·데이터 수집·학습·실행 | conda `lerobot` (Python 3.12, LeRobot 0.6.0) | `conda activate lerobot` |
| 게이트웨이(FastAPI)·시험, CamExample 예제 | 저장소 `.venv` (Python 3.11) | `source ~/Desktop/SO_ARM101/.venv/bin/activate` |

두 환경을 동시에 켜지 않는다. 바꿀 때는 먼저 끈다(`conda deactivate` / `deactivate`).

## 1. 가상환경 켜기·끄기

```bash
# conda lerobot 켜기
conda activate lerobot
# 위 명령이 "conda: command not found" 또는 activate 오류를 내면 (conda init 안 된 셸)
~/miniforge3/bin/conda init bash && source ~/.bashrc     # 한 번만 하면 이후 터미널에 계속 적용
source ~/miniforge3/bin/activate lerobot                 # init 없이 이 터미널에서만 켜기

conda deactivate                                         # 끄기
conda env list                                           # 환경 목록

# 저장소 .venv 켜기 / 끄기
cd ~/Desktop/SO_ARM101 && source .venv/bin/activate
deactivate
```

켜졌는지 확인: 프롬프트 앞에 `(lerobot)` 또는 `(.venv)`가 붙고, `which python` 경로가 그 환경 안을 가리킨다.

## 2. 장치 확인 (팔·카메라)

```bash
ls /dev/ttyACM*                      # 팔 2대 → ttyACM0, ttyACM1 (번호는 꽂는 순서로 바뀔 수 있음)
ls -l /dev/serial/by-id/             # 보드 고유 번호 ↔ ttyACM 번호
lerobot-find-port                    # 팔 하나를 뽑으면 사라진 포트를 알려 줌

ls /dev/video*                       # 웹캠 1대당 2개 생김, 영상은 짝수 번호
v4l2-ctl --list-devices              # 카메라 이름 ↔ /dev/video 번호
lerobot-find-cameras opencv          # 웹캠마다 사진을 찍어 outputs/captured_images/에 저장
lerobot-find-cameras realsense       # RealSense 이름·시리얼
```

2026-11-10 확인 값 (위·손목 배정은 오후에 다시 꽂은 뒤 사진으로 확인):

| 장치 | 값 | 바뀌지 않는 경로 |
|---|---|---|
| 팔로워암 | `/dev/ttyACM1` | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14030764-if00` |
| 리더암 | `/dev/ttyACM0` | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114649-if00` |
| 위 카메라 | `/dev/video0` | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3.4.1:1.0-video-index0` |
| 손목 카메라 | `/dev/video2` | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3.3:1.0-video-index0` |
| RealSense D435i | 시리얼 `243522072782` | 이 프로젝트에서는 쓰지 않음 (2026-11-10 결정) |

- 웹캠 두 대는 시리얼이 똑같아서(`SN0001`) by-id로 구분할 수 없다. by-path는 꽂은 USB 포트 기준이므로 항상 같은 포트에 꽂는다.
- 구성: 웹캠 2대 = 위(작업 영역 + 키트 박스) 1대 + 손목 1대. 다른 USB 포트에 꽂으면 경로가 바뀌므로 project/src/kitbot_exec/camera_settings.json의 by_path도 고친다. `lerobot-find-cameras opencv` 사진으로 정한 뒤 이 표에 적는다.

## 2-1. 카메라 설정 고정 (`.venv` 또는 lerobot, project/src에서)

```bash
cd ~/Desktop/SO_ARM101/project/src
python -m kitbot_exec.camera_lock apply        # 저장한 노출·화이트밸런스 적용. USB를 다시 꽂거나 재부팅한 뒤 매번
python -m kitbot_exec.camera_lock show         # 지금 카메라에 들어가 있는 값
python -m kitbot_exec.camera_lock tune top     # 조명이 바뀌었을 때 위 카메라 값을 새로 찾아 저장
python -m kitbot_exec.view_blocks --role top --color red   # 고정 설정 + 색 보정으로 블록 검출 화면
```

## 2-2. 학습 없이 집어서 바구니에 넣기 (conda lerobot, project/src에서)

단계별 진행 방법은 [Steps/S02_ActionPlan.md](Steps/S02_ActionPlan.md)에 있다. 먼저 위 카메라 라이브 창(view_blocks)을 닫고, 원격 조종(lerobot-teleoperate)도 끈다.

```bash
conda activate lerobot && cd ~/Desktop/SO_ARM101/project/src
python -m kitbot_exec.camera_lock apply
python -m kitbot_exec.teach                     # 1) h=HOME, b=BASKET, g=집기 직전 그리퍼, c=닫힘 그리퍼, p=그리퍼 끝 위치
python -m kitbot_exec.collect_calib --color purple # 2) 블록 9곳: d(픽셀) → 그리퍼 끝을 블록 윗면에 대고 r → ... → f(저장)
python -m kitbot_exec.pick --color purple --dry    # 3) 계산만
python -m kitbot_exec.pick --color purple --step   # 4) 단계마다 Enter
python -m kitbot_exec.pick --color purple --no-place --step   # 블록 위에서 멈춤 → 리더암으로 정확히 맞추면 어긋남 기록·자동 보정
```

프로그램이 끝나도 팔로워암은 힘을 유지한다(팔이 떨어지지 않게). 손으로 움직이려면 **팔을 받친 채** 힘을 푼다:

```bash
python -m kitbot_exec.release                  # Enter를 누르면 팔로워암 힘이 풀림 (팔이 떨어지니 먼저 손으로 받침)
```

잘 맞는 상태(화면 보정, 접근 보정, teach 자세, 카메라 설정, 팔 캘리브레이션)를 통째로 저장·복원:

```bash
python -m kitbot_exec.snapshot save 이름        # → project/src/calibration/snapshots/날짜_이름/
python -m kitbot_exec.snapshot list
python -m kitbot_exec.snapshot restore 폴더명    # 지금 파일은 var/before_restore/에 백업한 뒤 덮어씀
```

## 3. 캘리브레이션 (`conda activate lerobot`)

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=my_follower_arm
lerobot-calibrate --teleop.type=so101_leader  --teleop.port=/dev/ttyACM0 --teleop.id=my_leader_arm
```

- 처음 질문: Enter = 기존 파일을 그대로 모터에 씀, `c` + Enter = 새로 캘리브레이션.
- 순서: 모든 관절을 가동 범위 가운데에 두고 Enter → 손목 회전을 뺀 관절을 끝에서 끝까지 움직이고 Enter → `Calibration saved to ...`가 나오면 저장됨.
- Ctrl+C로 끝내면 저장되지 않는다.
- 파일 위치: `~/.cache/huggingface/lerobot/calibration/{robots/so_follower, teleoperators/so_leader}/<id>.json`
- 새로 캘리브레이션하면 저장소에도 복사한다(이력 보관): `cp -r ~/.cache/huggingface/lerobot/calibration/{robots,teleoperators} ~/Desktop/SO_ARM101/project/src/calibration/`

## 4. 원격 조종 (리더암 → 팔로워암)

```bash
# 팔만
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=my_follower_arm \
  --teleop.type=so101_leader  --teleop.port=/dev/ttyACM0 --teleop.id=my_leader_arm

# 카메라 포함 + Rerun 화면 (먼저 camera_lock apply)
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=my_follower_arm \
  --robot.cameras="{ wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}, top: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader  --teleop.port=/dev/ttyACM0 --teleop.id=my_leader_arm \
  --display_data=true
```

- 시작하면 팔로워암이 리더암 자세로 바로 움직인다. 두 팔을 비슷한 자세로 맞춘 뒤 시작한다.
- 끝내기: Ctrl+C.
- (참고, 이 프로젝트는 웹캠 2대만 씀) RealSense를 넣을 때는 `--robot.cameras`에 `realsense: {type: intelrealsense, serial_number_or_name: "243522072782", width: 640, height: 480, fps: 30}`를 추가한다(시험하지 않음).
- `--robot.id`와 `--teleop.id`는 캘리브레이션할 때 쓴 이름과 같아야 한다.

## 5. 데이터 수집 (`lerobot-record`)

```bash
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=my_follower_arm \
  --robot.cameras="{ wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}, top: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader  --teleop.port=/dev/ttyACM0 --teleop.id=my_leader_arm \
  --display_data=true \
  --dataset.repo_id=ssafy/kitbot_pick_test \
  --dataset.single_task="Pick up the block" \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=10 \
  --dataset.push_to_hub=false
```

| 키 | 동작 |
|---|---|
| `→` 또는 `n` | 이번 에피소드(시연 1회)를 일찍 끝내고 다음으로 |
| `←` 또는 `r` | 이번 에피소드를 버리고 다시 녹화 |
| `ESC` 또는 `q` | 녹화를 끝내고 영상 인코딩·저장 |

- 저장 위치: `~/.cache/huggingface/lerobot/<repo_id>/`. 같은 repo_id에 이어서 녹화하려면 `--resume=true`를 붙인다.
- `--dataset.push_to_hub=false`면 Hugging Face에 올리지 않고 이 PC에만 저장한다.
- 녹화한 에피소드 재생으로 확인: `lerobot-replay --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=my_follower_arm --dataset.repo_id=ssafy/kitbot_pick_test --dataset.episode=0`

## 6. 학습 (`lerobot-train`, ACT)

```bash
lerobot-train \
  --dataset.repo_id=ssafy/kitbot_pick_test \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/act_kitbot_pick_test \
  --job_name=act_kitbot_pick_test

# 끊긴 학습 이어서 하기
lerobot-train \
  --config_path=outputs/train/act_kitbot_pick_test/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

- 체크포인트: `outputs/train/<job>/checkpoints/<step>/pretrained_model/`, 마지막은 `checkpoints/last/`.
- GPU 사용 확인: 학습 중 다른 터미널에서 `nvidia-smi`. RTX 4050 6GB라 메모리가 부족하면 `--batch_size`를 줄인다.

## 7. 학습한 정책으로 팔 움직이기 (`lerobot-rollout`)

```bash
lerobot-rollout \
  --strategy.type=base \
  --policy.path=outputs/train/act_kitbot_pick_test/checkpoints/last/pretrained_model \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=my_follower_arm \
  --robot.cameras="{ wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}, top: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --task="Pick up the block" \
  --duration=60
```

- 카메라 이름(wrist/top)과 해상도는 데이터를 수집할 때와 똑같아야 한다.
- 처음에는 팔 주변을 비우고 손을 전원 가까이에 둔다. 멈출 때는 Ctrl+C.
- 이 절은 공식 문서 예시를 이 PC 값으로 바꾼 것이다. 이 PC에서 아직 실행하지 않았다.
