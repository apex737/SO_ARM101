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

## 2-3. 일반화 성능 실험 (conda lerobot, project/src에서)
**방식: v1 (2026-11-11).** 실험 방식이 바뀌면 이 절과 아래 "방식 변경 기록"을 함께 고친다.

블록을 처음 보는 자리에 놓고, ACT에 넘기는 것과 같은 상태(보정 없는 CV+IK 접근)가 얼마나 맞는지 잰다. 한 번 실행에 다음을 모두 한다.
1. HOME → 위 카메라로 블록 찾기 → 보정 없이 블록 위 40mm 접근
2. 손목 사진으로 접근 오차(mm·도)를 자동 측정 (S02에서 고친 손목 기준 `var/wrist_ref.json`)
3. 벌린 손가락 끝이 블록 윗면 위 약 15mm(명령, 실제 약 12mm)까지 수직으로 내려가 멈춤 → 눈 판정(선택) → 같은 길로 올라감 → HOME
4. 블록 자리 종류를 자동 기록: **평가 격자 안 / 격자 밖·보정점 범위 안 / 보정점 범위 밖**

```bash
source ~/miniforge3/bin/activate lerobot && cd ~/Desktop/SO_ARM101/project/src
export RUN="../evidence/$(date +%F)_s02"
s02() { python -m kitbot_exec.exp_s02 --run "$RUN" --color purple "$@"; }
gx() { python -m kitbot_exec.view_blocks --role top --color purple --calib kitbot_exec/calib_top.json --grid; s02 general --name "$1" "${@:2}"; }

gx 01            # 위 카메라 창에서 블록을 놓고 창 닫기(클릭 + q 또는 X) → 실험 시작
gx 02 --step     # 단계마다 Enter
gx 03 --no-ask   # 눈 판정 없이 사진 측정만
s02 report --prefix gen_     # 요약 (자리 종류별로 나눠 보여 줌)
```

**블록 놓는 법**
- 초록 원(g01~g09)에서 **2cm 이상 떨어진 곳**에 놓는다. 원에서 20mm 안이면 출력에 "처음 보는 자리로 치지 않음"이 뜬다.
- 각도는 −45°~45°에서 매번 다르게.
- 격자 안·격자 밖을 섞는다. 보정점 범위 밖은 위 카메라 계산이 범위 밖으로 추정하는 자리라 오차가 커질 수 있다(그걸 보려고 일부러 두는 것은 괜찮다).

**눈 판정 입력** (내려가서 멈췄을 때, 그리퍼 넓은 면 정면에서 본다. Enter만 누르면 건너뜀)

| 입력 | 뜻 |
|---|---|
| `c` | 블록이 두 손가락 가운데 |
| `f 2 14` | 고정(빨강) 손가락이 가까움. 그쪽 틈 2mm, 반대쪽 14mm |
| `m 2 14` | 움직이는(검정) 손가락이 가까움. 그쪽 틈 2mm, 반대쪽 14mm |

**결과 보는 법**
- 시행마다: `접근 오차 (손목 사진): ○○ mm, 각도 ○○°` → **10mm·10° 이하면 통과**(S02와 같은 기준). 마지막 줄에 자리 종류와 가장 가까운 격자·보정점 거리.
- 요약(`report --prefix gen_`): `approach.pass_rate`(전체), `by_region.<자리>.pass_rate`(자리별), `human_low`(눈 판정 목록). 파일: `$RUN/report_gen_.json`.
- 기록: `$RUN/trials.jsonl`(이름 `gen_…`), 사진: `var/exp_s02/frames/`(접근 높이 `*_gen_…jpg`, 내려간 자리 `*_low_gen_…jpg`).

**주의**
- 블록 윗면 아래로 내리지 않는다(`--clear`를 0 이하로 주지 않는다). 어긋남이 10mm에 가까운 자리에서는 손가락이 블록에 닿을 수 있다. 틈이 안 보이면 `--clear 8` 정도까지만.
- 실험 전 같은 날 한 번 S05 "세션 시작 점검"(g05에서 straddle → `c`)을 해 둔다. 장비가 움직였으면 결과가 의미 없다.
- 보정 파일(접근 보정 기록)은 읽지도 바꾸지도 않는다. 손목 기준 파일도 바꾸지 않는다.

**방식 변경 기록**
| 버전 | 날짜 | 바뀐 것 |
|---|---|---|
| v1 | 2026-11-11 | 처음 만듦: 보정 없는 접근 + 손목 사진 오차 + 내려가 보기(15mm) + 자리 종류 자동 분류. 판정 10mm·10° |

## 2-4. ACT 시작 자세 만들기 (`approach.py`, conda lerobot, project/src에서)
블록 위로 접근한 뒤 HOME으로 돌아가지 않고 그 자세에서 힘을 유지한 채 끝낸다(풀기: `python -m kitbot_exec.release`). 시행마다 시작 조건이 `var/approach_log.jsonl`에 한 줄씩 남는다.

```bash
python -m kitbot_exec.approach --color purple --step                                   # 보정 없이 40mm 접근 (기본)
python -m kitbot_exec.approach --color purple --handoff-clear 15                       # 40mm 접근 → 집게 끝이 블록 윗면 위 15mm(명령)까지 수직 하강
python -m kitbot_exec.approach --color purple --handoff-clear 15 --jitter-mm 15 --jitter-deg 15   # 시연용: 목표를 일부러 흔들기
```

| 옵션 | 뜻 |
|---|---|
| `--corr` | 접근 보정. 기본 `none`(S02 통과·ACT 인계 조건과 같음). `default` = 어제 리더암 보정 |
| `--handoff-clear MM` | 40mm 접근 뒤 더 내려가 넘기기. 5 이상만 허용(블록 윗면 아래로 안 감). 실제는 명령보다 2~3mm 낮음 |
| `--jitter-mm`, `--jitter-deg` | 목표를 반지름 MM 원 안, 각도 ±DEG 안에서 무작위로 흔듦. 계획이 안 되면 최대 5번 다시 뽑음. `--seed`로 재현 |
| `--step` | 이동 전마다 Enter |

**리더암 자동 맞춤 실험 (2026-11-11 추가, 실물 미확인)**
접근이 끝난 뒤 리더암 서보에 잠깐 힘을 넣어 팔로워 자세로 천천히 옮기고, 사람이 리더암을 잡으면 힘을 푼다. 녹화 시작 순간 팔로워가 튀지 않게 하려는 것.
```bash
python -m kitbot_exec.approach --color purple --handoff-clear 15 --align-leader --step                 # 맞추기만
python -m kitbot_exec.approach --color purple --handoff-clear 15 --align-leader --teleop-s 10 --step   # 맞춘 뒤 원격 조종 10초
```
1. 리더암 주변에 손·물건이 없게 둔다. **리더암이 스스로 움직인다.** 서보 힘은 40%(`--leader-torque 400`)로 낮춰 둔다.
2. 리더암이 팔로워 자세에 도착하면 "리더암을 손으로 잡으세요"가 뜬다 → 잡고 Enter → 리더암 힘이 풀린다.
3. 출력 세 숫자를 본다: 맞추기 전 관절 차이 / 힘 켠 채 차이 / **힘 푼 뒤 차이(그리퍼 제외)**. `--teleop-s`를 주면 원격 조종 처음 0.5초 동안의 **팔로워 관절 변화 최대**도 나온다.

| 결과 | 판단 (추정 기준) |
|---|---|
| 힘 켠 채 차이가 크다(5° 넘음) | 리더암 서보 힘이 모자람 → `--leader-torque 600`으로 한 번 더 |
| 힘 푼 뒤 차이 3° 이하, 원격 조종 처음 0.5초 변화 3° 이하 | 자동 맞춤 사용 가능 → S05-1 녹화 도구에 넣는다 |
| 힘 푼 뒤 차이가 큼 | 힘을 풀 때 리더암이 처짐 → 사람이 잡는 방식(관절 차이 표시)으로 간다 |

## 2-5. ACT 시연 여러 개 이어서 녹화 (`record_act.py`, conda lerobot, project/src에서)
S05 시연 녹화 도구. 에피소드마다 접근(흔들기 + 하강)과 리더암 자동 맞춤을 한 뒤 원격 조종을 녹화하고, LeRobot 데이터셋으로 저장한다. 저장한 데이터셋은 6절 `lerobot-train`으로 바로 학습한다.

```bash
python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --episodes 10 --color purple   # 처음 (집을 색 지정)
python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --episodes 10 --resume     # 같은 데이터셋에 이어서
python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --episodes 10 --resume --jitter-prob 0.5 --jitter-mm 10 --jitter-deg 10
```

**에피소드 하나의 순서**
1. "블록을 새 자리에 놓고 Enter" → 블록을 놓는다(평가 격자 원 g01~g09에서 2cm 안은 피한다, S05 유의사항 9). `q` Enter = 녹화 끝.
   - 이번 에피소드부터 다른 색을 집으려면 Enter 대신 색 이름을 친다(예: `blue` Enter). 바꾼 색은 다음 에피소드에도 유지된다.
   - 고를 수 있는 색: `red`, `orange`, `yellow`, `green`, `blue`, `purple` (2026-11-11 위 카메라 실측으로 범위를 나눔, 6색을 함께 두고 각각 1개씩 잡히는 것 확인).
   - **여러 색 블록을 함께 둬도 된다.** 단, 작업 영역 안에 집을 색이 2개 이상이면 접근하지 않고 그 에피소드를 건너뛴다(잘못 집기 방지). 바구니 안 블록은 작업 영역 밖이라 세지 않는다.
   - 블록끼리는 붙이지 않는다(같은 색이 붙으면 한 덩어리). 흔들기 폭을 생각해 집을 블록과 옆 블록은 약 6cm 이상 띄운다(추정).
2. 자동: HOME → 위 카메라로 블록 찾기 → 흔든 목표로 40mm 접근 → 블록 윗면 위 15mm까지 수직 하강.
3. 자동 인계 판단: 관절 오차 4° 이하 + 손목 카메라에 블록 보임. 아니면 그 에피소드를 건너뛴다.
4. 리더암 자동 맞춤: **리더암이 스스로 움직인다(손 치우기)** → "리더암을 손으로 잡으세요" → 잡고 Enter → 힘이 풀린다.
5. **녹화 중**: 리더암으로 보정 → 집기 → 바구니 위에서 놓기 → HOME 자세. 키(터미널이 아니어도 됨):

| 키 | 동작 |
|---|---|
| `n` 또는 `→` | 이번 에피소드 끝내고 저장 |
| `r` 또는 `←` | 이번 에피소드 버리고 다시 (실패한 시연) |
| `q` 또는 `Esc` | 이번 에피소드 저장하고 전체 종료 |

   `--episode-s`(기본 30초)가 지나면 자동으로 끝내고 저장한다.
6. 자동: 팔로워를 HOME으로 (녹화 밖) → 1로.

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--color` | purple | 집을 블록 색(시작 값). 에피소드마다 바꿀 수 있음 |
| `--jitter-mm`, `--jitter-deg` | 15, 15 | 접근 목표 흔들기 반지름[mm]·각도[°] |
| `--jitter-prob` | 1.0 | 에피소드마다 흔들 확률. 0.5면 절반은 CV 접근 그대로 |
| `--handoff-clear` | 15 | 넘기는 높이(블록 윗면 위, 명령). 0이면 40mm에서 넘김 |
| `--max-arrival-deg` | 4 | 인계 판단 관절 오차 한도 |
| `--leader-torque` | 400 | 리더암 자동 맞춤 때 서보 힘(0~1000) |
| `--max-leader-gap` | 5 | 리더암 힘 푼 뒤 차이가 이보다 크면 녹화 전에 묻는다 |
| `--step` | 꺼짐 | 이동 전마다 Enter |

- 저장 위치: `~/.cache/huggingface/lerobot/local/kitbot_act_v1/`. 에피소드별 시작 조건(블록 위치, 흔든 값, 인계 판단, 리더암 차이, 녹화 Hz): 같은 폴더의 `kitbot_episode_meta.jsonl`.
- 출력의 "평균 ○○Hz"가 27 아래로 자주 나오면 알려 줄 것(영상과 동작 시간이 어긋남).
- 영상은 camera_settings.json 고정값 + 색 보정을 적용한 RGB다. 학습한 정책을 실행할 때도 같은 처리를 써야 한다(S05-4에서 같은 카메라 코드 사용).
- 학습: 6절 명령에서 `--dataset.repo_id=local/kitbot_act_v1`, `--output_dir=outputs/train/act_kitbot_v1`, `--job_name=act_kitbot_v1`로 바꿔 실행.

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

> S05 ACT 시연은 CV+IK 접근이 끝난 자세에서 시작해야 해서, 이 명령 대신 2-5절 `record_act.py`를 쓴다. 넘어가는 조건과 학습 유의사항: [Steps/S05.md](Steps/S05.md). 아래 명령은 원격 조종 녹화 연습용.

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
