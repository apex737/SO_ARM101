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
**방식: v2 (2026-11-12).** 실험 방식이 바뀌면 이 절과 아래 "방식 변경 기록"을 함께 고친다.

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
- v2부터 보정점 범위 밖(작업 영역 + 25mm 밖 포함)도 접근한다. 바구니 앞 테두리 선 너머(바구니 안)만 빠진다. 범위 밖이면 접근 출력에 '보정점 범위 밖 N mm' 경고가 나온다.

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
| v2 | 2026-11-12 | 블록 자리 제한 변경: '보정점 범위 + 25mm' 밖도 접근, 바구니 앞 테두리 선 너머만 제외(pick.BASKET_FRONT). 범위 밖 거리를 경고·기록. 판정 기준은 그대로 |

- 학습 방식과의 비교(같은 격자 18자리, 순수 IK vs IK+ACT)는 2-7.

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
python -m kitbot_exec.record_act --repo-id local/kitbot_act_test --episodes 2 --color purple --step   # 맨 처음 시험 (본 데이터와 분리)
# 본 녹화: 50개 = 안 흔듦 25 + 흔듦 25, 25개마다 쉬는 시간. 끊기면 같은 명령을 다시 치면 이어서 남은 만큼만
python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --goal 50 --pause-every 25 --color purple
# 오늘은 25개만, 나머지는 다음에 (같은 명령 다시)
python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --goal 50 --episodes 25 --color purple
```

**에피소드 하나의 순서**
1. "블록을 새 자리에 놓고 Enter" → 블록을 놓는다(평가 격자 원 g01~g09에서 2cm 안은 피한다, S05 유의사항 9). `q` Enter = 녹화 끝.
   - 이번 에피소드부터 다른 색을 집으려면 Enter 대신 색 이름을 친다(예: `blue` Enter). 바꾼 색은 다음 에피소드에도 유지된다.
   - 고를 수 있는 색: `red`, `orange`, `yellow`, `green`, `blue`, `purple` (2026-11-11 위 카메라 실측으로 범위를 나눔, 6색을 함께 두고 각각 1개씩 잡히는 것 확인).
   - **여러 색 블록을 함께 둬도 된다.** 단, 테이블에 집을 색이 2개 이상이면 접근하지 않고 그 에피소드를 건너뛴다(잘못 집기 방지). 바구니 안 블록은 세지 않는다(바구니 앞 테두리 선 너머).
   - 블록끼리는 붙이지 않는다(같은 색이 붙으면 한 덩어리). 흔들기 폭을 생각해 집을 블록과 옆 블록은 약 6cm 이상 띄운다(추정).
2. 자동: HOME → 위 카메라로 블록 찾기 → 흔든 목표로 40mm 접근 → 블록 윗면 위 15mm까지 수직 하강.
3. 자동 인계 판단: 관절 오차 4° 이하 + 손목 카메라에 블록 보임. 아니면 그 에피소드를 건너뛴다.
4. 리더암 자동 맞춤: **리더암이 스스로 움직인다(손 치우기)** → "리더암을 손으로 잡으세요" → 잡고 Enter → 힘이 풀린다.
   **손 맞춤** (팔로워는 그 자리에 멈춰 있음): 관절별로 "리더암을 더 움직여야 할 양(팔로워 − 리더)"이 실시간으로 나온다.
   ```
   pan  +0.4  lift  -1.2  elbow  +4.1* wflex  +0.3  roll  -2.9  grip  +1.0   (* = 더 맞추기)
   ```
   `*` 표시가 모두 사라지면 "→ 맞음, Enter = 녹화 시작". 숫자가 줄어드는 쪽으로 리더암을 움직이면 된다.

| 키 (Enter 없이 바로) | 동작 |
|---|---|
| `Enter` 또는 스페이스 | 녹화 시작 (모든 관절이 허용 범위 안일 때만) |
| `f` | 허용 범위를 넘은 채로 그래도 시작 |
| `a` | 리더암 자동 맞춤 다시 |
| `s` | 이 에피소드 건너뛰기 (팔로워 HOME) |

   키를 누르려고 리더암에서 손을 떼면 리더암이 처질 수 있다. 한 손으로 리더암을 받친 채 누르거나, `--auto-start-s 1`(허용 범위 안을 1초 유지하면 자동 시작)을 쓴다.
5. **녹화 중**: 리더암으로 보정 → 집기 → 바구니 위에서 놓기 → HOME 자세. 키(터미널이 아니어도 됨):

| 키 | 동작 |
|---|---|
| `n` 또는 `→` | 이번 에피소드 끝내고 저장 |
| `r` 또는 `←` | 이번 에피소드 버리고 다시 (실패한 시연) |
| `q` 또는 `Esc` | 이번 에피소드 저장하고 전체 종료. HOME으로 간 뒤 한 번 더 묻는다(Enter = 종료, `c` = 계속) |

   `--episode-s`(기본 30초)가 지나면 자동으로 끝내고 저장한다.
   녹화 중 누른 키는 녹화가 끝나면 터미널 입력에서 지운다(다음 질문의 답으로 들어가지 않게). 끝날 때 "끝난 이유"를 출력한다.
6. 자동: 팔로워를 HOME으로 (녹화 밖) → 1로.

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--color` | purple | 집을 블록 색(시작 값). 에피소드마다 바꿀 수 있음 |
| `--jitter-mm`, `--jitter-deg` | 10, 10 | 접근 목표 흔들기 반지름[mm]·각도[°] (S05 D03) |
| `--goal` | 없음 | 데이터셋 전체 목표 개수. 다 차면 멈추고, 이미 차 있으면 장비를 켜지 않고 끝 |
| `--episodes` | 목표까지 (목표 없으면 10) | 이번에 저장할 개수 |
| `--pause-every` | 0 | 이번에 이만큼 저장할 때마다 쉬는 시간(Enter = 계속, q = 끝) |
| `--jitter-mode` | balanced | 저장된 에피소드 기준으로 흔듦·안 흔듦 중 적은 쪽을 고른다 → 몇 번에 나눠도, 실패·버림이 있어도 차이 1 이하. `random`이면 `--jitter-prob` 확률 |
| `--handoff-clear` | 15 | 넘기는 높이(블록 윗면 위, 명령). 0이면 40mm에서 넘김 |
| `--max-arrival-deg` | 4 | 인계 판단 관절 오차 한도 |
| `--leader-torque` | 400 | 리더암 자동 맞춤 때 서보 힘(0~1000) |
| `--start-gap` | 3 | 손 맞춤: 팔 관절 리더−팔로워 차이 허용[°]. 이 안이어야 Enter로 시작 |
| `--start-grip-gap` | 3 | 손 맞춤: 그리퍼 차이 허용(0~100 값) |
| `--auto-start-s` | 0 (끔) | 손 맞춤: 허용 범위 안을 이 시간[초] 유지하면 키 없이 시작 |
| `--step` | 꺼짐 | 이동 전마다 Enter |

- **이어서 녹화는 자동**: 데이터셋 폴더가 이미 있으면 `--resume` 없이도 이어서 쌓는다(가짜 데이터로 만들기 1 → 이어서 2 → 다시 읽기 3개 확인, 2026-11-11). 처음부터 다시 하려면 폴더를 지우거나 새 이름.
- 흔들기 설정은 한 데이터셋 안에서 바꾸지 않는다. 기존 에피소드와 흔들기 폭·넘기는 높이가 다르면 장비를 켜기 전에 거부한다. 바꾸려면 새 이름(`kitbot_act_v2`)으로.
- 입력 칸에 다음 에피소드가 흔듦인지 안 흔듦인지와 진행 상황(예: `데이터셋 27/50 (흔듦 13 · 안 흔듦 14)`)이 나온다.

- 저장 위치: `~/.cache/huggingface/lerobot/local/kitbot_act_v1/`. 에피소드별 시작 조건(블록 위치, 흔든 값, 인계 판단, 리더암 차이: 자동 맞춤 뒤·녹화 시작 때·손 맞춤 시간, 녹화 Hz): 같은 폴더의 `kitbot_episode_meta.jsonl`.
- 출력의 "평균 ○○Hz"가 27 아래로 자주 나오면 알려 줄 것(영상과 동작 시간이 어긋남).
- HOME에 도착하면 **1초쯤 뒤 바로 `n`**. 시험 녹화 ep0은 HOME에서 9초를 그냥 서 있어 전체의 40%가 정지 화면이었다(학습 시간 낭비). 실행 도구는 HOME 근처 1초면 끝으로 본다.
- 블록을 쥐고 있는 동안 그리퍼 명령이 '블록 폭 − 8'로 잘리는 것은 정상이다(팔로워는 블록에서 멈추고 리더는 더 닫힘). "잘림" 경고는 팔 관절만 센다.
- 영상은 camera_settings.json 고정값 + 색 보정을 적용한 RGB다. 학습한 정책을 실행할 때도 같은 처리를 써야 한다 → 2-6 `run_act.py`가 같은 카메라 코드를 쓴다.
- 학습: 6절 "이 과제 데이터셋" 명령.

**녹화 영상 확인**
```bash
# ① 이 PC에서 바로 (올리지 않음, Rerun 창)
lerobot-dataset-viz --repo-id local/kitbot_act_test --episode-index 0
# ② Hugging Face에 비공개로 올려 웹에서 (hf auth whoami로 로그인 확인. images/는 임시 사진이라 뺀다)
hf upload apex737/kitbot_act_test ~/.cache/huggingface/lerobot/local/kitbot_act_test . --repo-type dataset --private --exclude "images/*"
hf repo tag create apex737/kitbot_act_test v3.0 --repo-type dataset   # LeRobot이 버전 태그로 데이터셋을 찾는다
```
- ②는 웹 페이지 `huggingface.co/datasets/apex737/kitbot_act_test` → Files → `videos/` 아래 mp4를 누르면 재생된다.
- LeRobot 데이터셋 보기 페이지(`huggingface.co/spaces/lerobot/visualize_dataset`)는 관절 그래프와 영상을 함께 보여 주지만, 공개 데이터셋만 읽는 것으로 안다(확인 전). 공개로 바꾸면 작업 공간 영상이 누구에게나 보인다.
- 이어서 녹화한 뒤에는 ②의 `hf upload`를 다시 하면 바뀐 파일만 올라간다.

## 2-6. 학습한 ACT 정책 실행·평가 (`run_act.py`, conda lerobot, project/src에서)
S05-4 실행 도구. 녹화(2-5)와 같은 접근·인계 판단·카메라 처리를 거친 뒤 정책이 팔을 움직이고, 끝·실패를 스스로 판정해 멈춘다. 7절 `lerobot-rollout`은 이 과제에 쓰지 않는다(접근 단계가 없고, 카메라 색 보정이 녹화 때와 다름).

```bash
python -m kitbot_exec.run_act --policy outputs/train/act_kitbot_v1_check/checkpoints/005000/pretrained_model --name check_5k --trials 3 --step   # 흐름 점검
python -m kitbot_exec.run_act --policy outputs/train/act_kitbot_v1/checkpoints/<단계>/pretrained_model --name act_v1 --color purple          # 비교 실험 18회 (2-7)
# 학습 중에 실물 시험: 학습을 잠깐 멈추고(GPU 비우기) → 시험 → 꼭 다시. 학습 결과는 그대로, 끝나는 시각만 늦어짐
python -m kitbot_exec.train_ctl pause      # status = 상태 보기
python -m kitbot_exec.train_ctl resume
# 앙상블 켜기 (매 프레임 다시 예측해 겹친 예측을 평균, ACT 논문 값 0.01). 기록 이름을 따로 두어 끈 것과 비교
python -m kitbot_exec.run_act --policy outputs/train/act_kitbot_v1_check/checkpoints/020000/pretrained_model --name act_20k_te --color purple --temporal-ensemble 0.01
```

**시행 하나의 순서**
1. "블록을 gNN에 놓고 Enter" → 위 카메라로 자리 확인(격자점에서 15mm 넘으면 옮길 양을 알려 줌). 격자 g01→g09를 2바퀴 안내한다. `--free`면 아무 자리. (색 이름 = 색 바꾸기, `q` = 끝)
2. 자동: HOME → 접근 → 블록 윗면 위 15mm까지 하강 (녹화와 같음, 흔들기는 기본 0) → 인계 판단(관절 오차 4° 이하 + 손목 카메라에 블록 보임).
3. 정책 첫 명령과 지금 자세 차이를 보여 준다. 10°보다 크면 실행할지 묻는다(학습이 덜 됐거나 입력이 녹화 때와 다를 때).
4. **정책 실행**: 손을 전원 가까이. `n` / `q` / `Esc` / `→` = 바로 멈춤 (`q`·`Esc`는 이번 시행 뒤 전체 종료할지 묻는다). 아래 중 하나면 스스로 멈춘다.

| 판정 | 뜻 | 멈춘 뒤 |
|---|---|---|
| HOME 도착 | HOME 자세 8° 안에 1초 머묾 = 정상 끝 | 그대로 |
| 빈손 | 닫으라는 명령인데 끝까지 닫힘(블록 없음) 0.5초 | 그리퍼 열고 HOME |
| 옮기다 떨어뜨림 | 잡은 뒤 놓기 전에 끝까지 닫힘 | 그리퍼 열고 HOME |
| 테이블 아래 명령 | 명령한 그리퍼 끝이 테이블보다 10mm 넘게 아래 → 그 명령은 보내지 않음 | Enter 뒤 HOME |
| 시간 초과 | 30초 | Enter 뒤 HOME (쥐고 있으면 HOME에서 놓음) |

5. 끝 사진(위·손목) 저장 → 사람 판정: 2-7의 코드(`s`/`g`/`d`/`b`/`x`/`?`). 인계 판단에서 막혀 정책을 못 돌린 경우도 한 시행으로 기록한다(순수 IK와 공정 비교).
6. 끝에 "성공 ○/○" 출력.

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--policy` | (필수) | `pretrained_model` 폴더 |
| `--name` | run_날짜_시각 | 기록 파일 `var/compare/<name>.jsonl`, 사진 `var/compare/<name>/` |
| `--trials` | 18 | 시행 수 (격자 9곳 × 2) |
| `--free` | 꺼짐 | 격자 안내·자리 확인 없이 아무 자리 |
| `--place-tol` | 15 | 놓은 블록과 격자점 거리 허용[mm] |
| `--max-step-deg` | 8 | 한 프레임(1/30초)에 관절이 움직일 수 있는 최대[°]. 녹화와 같게 둔다. 낮추면 블록을 쥐는 힘(리더가 블록 폭보다 8 더 닫힌 값)과 돌아가는 속도가 시연과 달라진다 |
| `--max-s` | 30 | 정책 실행 최대 시간[초] |
| `--floor-mm` | 10 | 테이블 아래 명령 한도[mm] |
| `--home-tol`, `--home-hold-s` | 8, 1 | HOME 도착 판정 |
| `--n-action-steps` | 학습값 | 한 번 예측한 동작 묶음에서 몇 개 실행 후 다시 예측 |
| `--temporal-ensemble` | 꺼짐 | 예: `0.01`. 매 프레임 예측하고 겹친 예측을 평균(ACT 논문 방식). 동작이 떨리면 시험 |
| `--jitter-mm`, `--jitter-deg` | 0, 0 | 접근 흔들기. 평가는 0, 어긋남에 강한지 볼 때만 |
| `--step` | 꺼짐 | 이동·정책 시작 전마다 Enter |

- 기록 한 줄: 정책 경로, 블록 자리와 구역(평가 격자 안 / 보정점 범위 안 / 밖), 접근 기록, 자동 판정(멈춘 이유, 잡은 시각, 놓은 시각·자세, 명령 끝 최저 높이, Hz, 제한 걸린 프레임), 사람 판정.
- "제한에 걸림" 프레임이 많으면 정책이 급하게 움직이려 한 것. "늦음" 프레임이 많으면 GPU 계산이 30Hz를 못 따라간 것(알려 줄 것).
- 위 카메라를 뺀 정책(`lerobot-edit-dataset ... remove_feature`로 만든 데이터셋으로 학습)도 그대로 실행된다. 정책이 받는 입력만 넘긴다.
- 정책 불러오기·한 단계 실행은 학습 안 한 작은 ACT로 확인했다(2026-11-11, GPU 한 단계 약 3ms, 새 묶음 예측 때 최대 25ms). 학습한 정책·실물로는 아직 실행하지 않았다.

### 디퓨전 정책 실행 (`run_diffusion.py`)
Kaggle(T4 2장)에서 20,000단계 학습한 디퓨전 정책. `run_act.py`와 같은 흐름·판정·기록이고 기본값만 디퓨전용(정책 경로, 기록 이름 `diffusion_20k`, 시간 제한 60초).

```bash
python -m kitbot_exec.run_diffusion --color purple --trials 3 --step     # 처음
python -m kitbot_exec.run_diffusion --color purple                       # 18회 → var/compare/diffusion_20k.jsonl
python -m kitbot_exec.run_diffusion --color purple --name diffusion_20k_n15 --n-action-steps 15   # 멈춤 횟수 절반
# 압축 다시 풀기 (학습 상태 2.2GB 제외)
unzip -o ~/Downloads/diffusion_latest_020000_*.zip -x "checkpoint/training_state/*" -d outputs/train/diffusion_kaggle_20k
```

- 모델: 데이터 `apex737/kitbot_act_v1` 50개 전부(불량 4개를 걸러 내기 전, 10%는 eval), 관찰 2프레임 → 16프레임 예측 → 8프레임 실행, 영상 240×320으로 줄여 입력, DDIM 10단계. 학습 때 eval loss 0.041(2k) → 0.015(20k).
- **움직임이 끊긴다(정상)**: 8프레임마다 새로 예측하고 한 번에 약 0.26초(이 PC 실측) → 그동안 팔이 멈춤, 평균 약 17Hz(ACT는 30Hz). "늦은 프레임" 경고가 매번 나온다. `--n-action-steps 15`면 약 20Hz.
- `--num-inference-steps 5`: 예측 약 0.13초, 대신 동작이 달라짐. 16비트 연산은 효과 없음(가중치만 16비트로 바꿔도 0.20초).
- `--temporal-ensemble`은 ACT 전용.

### 큐 실행: 색·개수를 차례로 (`--queue`, 세 방식 공통)
```bash
python -m kitbot_exec.eval_ik       --name q_ik   --queue "orange 2, yellow 1"          # 순수 IK
python -m kitbot_exec.run_act --policy outputs/train/act_kitbot_v1_check/checkpoints/020000/pretrained_model --name q_act --queue "orange 2, yellow 1"   # IK+ACT
python -m kitbot_exec.run_diffusion --name q_diff --queue "orange 2, yellow 1, blue"   # IK+디퓨전
```
- `"orange 2, yellow 1"` = 주황을 **최대 2개** 집고 → 노랑을 최대 1개. 개수를 안 쓰거나 `all`이면 그 색이 **안 보일 때까지**(`"blue"`, `"blue all"`).
- 다음 명령으로 넘어가는 때: ① 목표 개수를 채움 ② 그 색이 테이블에 더 안 보임 ③ 연속 2번 실패(닿지 않는 블록 하나로 끝없이 되풀이하지 않게).
- 같은 색이 여러 개면 로봇 받침대에서 가까운 것부터 집는다. 바구니 안 블록은 세지 않는다.
- 받침대에 가까운 블록(약 110mm 안팎): 기본 높이로 계획이 안 되면 순수 IK는 손목을 90° 돌린 후보와 낮은 접근 높이(30, 25mm)를 자동으로 시도하고 `※`로 알려 준다. 그래도 안 되면 `plan_failed`(연속 2번이면 다음 명령).
- **묻지 않고 이어서 실행한다**: 블록 놓기 Enter·사람 판정·"HOME으로 Enter"가 없다. 멈추려면 Ctrl+C(정책 실행 중에는 `n`/`q`/`Esc`도). 처음에는 손을 전원 가까이.
- 기록: 시행마다 `var/compare/<name>.jsonl`에 자동 결과와 끝 사진이 남고 판정은 `?`(사람 판정 없음). 끝에 `큐 결과: orange 2/2 (목표 2개 채움), yellow 0/1 (더 안 보임)`.
- '집음'으로 세는 기준: 순수 IK = 잡아서 놓고 HOME까지 끝남(`placed`), ACT·디퓨전 = 잡은 것이 확인되고 HOME에서 끝남. 바구니에 실제로 들어갔는지는 보지 않는다(끝 사진으로 확인).
- ACT·디퓨전은 시연이 모두 "그 색 1개" 장면이었다. 같은 색이 여러 개 보이는 장면은 학습한 적이 없어 성능이 다를 수 있다(미확인).

## 2-7. 비교 실험: 순수 IK vs IK+ACT (conda lerobot, project/src에서)
일반화 성능 비교: 블록을 **작업 영역 아무 곳**에 놓고 두 방식을 시험한다. 블록 자리는 접근할 때 위 카메라가 찾은 값으로 자동 기록되고, 가장 가까운 시연(학습에 쓴 41개) 자리까지 거리로 묶어 본다(시연<20mm / 시연<40mm / 시연≥40mm). 같은 판정 코드·같은 형식으로 기록해 한 표로 비교한다.

```bash
# ① 순수 IK (학습 없음, GPU 안 씀 → 학습 중에도 가능). pick.py를 접근 보정 없이 반복
python -m kitbot_exec.eval_ik --name ik_v1 --color purple
# ② IK+ACT (같은 접근 → 블록 위 15mm에서 ACT가 보정·집기·바구니·HOME)
python -m kitbot_exec.run_act --policy outputs/train/act_kitbot_v1/checkpoints/<단계>/pretrained_model --name act_v1 --color purple
# ③ 비교 표 (성공률, 실패 종류별 수, HOME 복귀, 평균 시간, 시연 거리별 판정)
python -m kitbot_exec.compare var/compare/ik_v1.jsonl var/compare/act_v1.jsonl
# 끊겼을 때 이어서: 이미 기록된 수를 --start로 (예: 7개 했으면). 같은 --name이면 같은 파일에 이어 붙음
python -m kitbot_exec.eval_ik --name ik_v1 --color purple --start 7
python -m kitbot_exec.run_act --policy <같은 체크포인트> --name act_v1 --color purple --start 7
```

- **색 바꾸기**: 두 도구 모두 "블록을 놓고 Enter" 칸에서 Enter 대신 색 이름(`blue` 등)을 치면 그 시행부터 바뀐다. 기록에 시행마다 색이 남는다.
- 시행 수는 `--trials`(기본 18). 두 방식 차이가 작으면 `--start 18 --trials 36`으로 이어서 늘린다(S05 D06).
- **어디까지 놓을 수 있나** (2026-11-12부터): 바구니 앞 테두리보다 로봇 쪽 테이블이면 어디든. 바구니 안 블록은 세지 않는다. 보정점 범위(x 87~150, y 38~239mm) 밖이면 "보정점 범위 밖 N mm" 경고와 함께 접근하고 기록(`outside_calib_mm`)에 남는다. 팔이 닿지 않는 자리는 계획 단계에서 거부.
  - **같은 색이 여러 개일 때**: 순수 IK(`pick.py`, `eval_ik`)는 로봇 받침대에서 가장 가까운 블록을 골라 집는다(2026-11-12). 하나 집고 다시 실행하면 그다음 가까운 것. IK+ACT(`run_act`)와 녹화(`record_act`)는 그대로 거부한다(시연이 모두 그 색 1개 장면이라). 같은 색 블록이 서로 붙어 있으면 한 덩어리로 보여 잘못 집을 수 있으니 띄워 둔다.
  - 블록을 못 찾으면 이유가 나온다: 테이블에 같은 색 2개 / 바구니 쪽에서만 보임(좌표·거리) / 안 보임.
  - 바구니를 옮기면 바구니 앞 테두리 선(`pick.BASKET_FRONT`)을 다시 재야 한다.
- `--grid`: S02 평가 격자 g01~g09를 순서대로 안내하고 위 카메라로 자리를 확인한다(격자점에서 15mm 넘으면 다시 놓기). S02 결과와 같은 자리로 비교할 때만.

**같게 맞추는 것**
- 자리: 아무 곳이되 두 방식이 비슷하게 퍼지게 놓는다(작업 영역 가운데만, 가장자리만 몰리지 않게). 시연 자리 바로 위만 고르지 않는다. 결과 표의 시연 거리별 칸 수로 퍼짐을 확인.
- 접근: 둘 다 접근 보정 없음(`pick.py --no-correction` = run_act의 접근).
- 색·조명·바구니: 같은 색 블록 1개, 바구니는 테이프로 고정, 같은 날 연달아.
- 시행 수: 인계 판단에서 막히거나 접근이 실패한 IK+ACT 시행도 한 시행으로 센다(빼면 IK+ACT 성공률이 부풀려짐).

**판정 코드 (사람이 눈으로, 시행이 끝나고 팔이 HOME에 간 뒤)**

| 코드 | 뜻 |
|---|---|
| `s` | 성공: 블록 전체가 바구니 안(바닥이든 다른 블록 위든), 시행 중 사람이 손대지 않음 |
| `g` | 못 잡음: 들어 올릴 때 블록이 그리퍼에 없음 |
| `d` | 떨어뜨림: 잡아서 옮기다가 바구니에 닿기 전에 떨어짐 |
| `b` | 바구니 밖·테두리: 놓았지만 바구니 밖이거나 테두리에 걸쳐 있음 |
| `x` | 개입: 사람이 손대거나 비상 정지(충돌·위험) |
| `?` | 애매: 끝 사진으로 나중에 판정 (성공률 계산에서 빠지고 따로 셈) |

**애매할 때 규칙** (판정 전에 정해 둠, S05 D06)
- 테두리에 걸쳐 일부만 안 → `b`. 바구니 바닥이나 안쪽 블록에 닿아 있어야 성공.
- 놓친 블록이 튕겨서 우연히 들어감 → `s`, 메모 `튕김`. 다시 집어서 성공 → `s`, 메모 `재시도`. (결과 기준, 메모로 따로 셀 수 있게)
- 바구니에는 넣었는데 HOME으로 못 돌아옴 → `s`. HOME 여부는 자동 기록(`home_ok`)으로 따로 센다.
- 사람이 조금이라도 손대면 결과와 상관없이 `x`.
- 그래도 모르겠으면 `?` → 매 시행 끝에 저장한 사진 `var/compare/<name>/tNN_gNN_top.jpg`, `_wrist.jpg`로 나중에 판정.

- 실험을 시작할 때 위 판정 규칙이 화면에 한 번 나오고, 판정 칸에서 `h`를 치면 다시 나온다.
- 판정 뒤에 메모를 붙일 수 있다: `b 테두리 왼쪽`, `s 튕김`.
- 순수 IK 기록에는 pick.py 자동 결과(`placed`/`empty` 등, 그리퍼 값)도, IK+ACT 기록에는 멈춘 이유·잡은 시각·Hz 등도 같이 남는다.
- 두 방식 모두 끝에 HOME에서 그리퍼를 닫는다(시연과 같음, HOME 자세의 그리퍼 값). IK+ACT는 닫기 전 정책이 남긴 그리퍼 값을 `gripper_at_end`로 기록한다(정책이 시연처럼 스스로 닫았는지).
- 평균 시간은 접근 시작부터 HOME까지(판정 입력 제외). `--step`을 쓰면 늘어나니 본 비교에서는 쓰지 않는다.

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

**이 과제 데이터셋 (2-5에서 녹화, project/src에서 실행)**

학습 데이터는 원본 `kitbot_act_v1`(50개)에서 불량 ep0·1·4·8을 뺀 `kitbot_act_v1_clean`(46개, 23,295프레임, 흔듦 23 · 안 흔듦 23)이다. 거르는 명령과 이유: S05.md 검증 이력 2026-11-12.

```bash
# 거르기 (한 번만, 이미 해 둠. 원본은 그대로)
lerobot-edit-dataset --repo_id local/kitbot_act_v1 --root ~/.cache/huggingface/lerobot/local/kitbot_act_v1 \
  --new_repo_id local/kitbot_act_v1_clean --new_root ~/.cache/huggingface/lerobot/local/kitbot_act_v1_clean \
  --operation.type delete_episodes --operation.episode_indices "[0, 1, 4, 8]"

# 학습에 쓸 에피소드 = eval용 5개(6, 13, 22, 27, 36)를 뺀 41개. eval loss를 따로 보기 위함 (S05 D05)
EPS="[0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40, 41, 42, 43, 44, 45]"

# ① 흐름 점검용 짧은 학습 (5,000단계, 약 45분, 1,000단계마다 저장) → 2-6 run_act로 실행 흐름만 확인
lerobot-train --dataset.repo_id=local/kitbot_act_v1_clean --dataset.root=$HOME/.cache/huggingface/lerobot/local/kitbot_act_v1_clean \
  --dataset.episodes="$EPS" --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --steps=5000 --save_freq=1000 --output_dir=outputs/train/act_kitbot_v1_check --job_name=act_kitbot_v1_check \
  > train_act_kitbot_v1_check.log 2>&1

# ② 본 학습 (50,000단계, 약 7시간, 5,000단계마다 저장). 터미널을 닫아도 계속되게 nohup + 기록 파일
nohup lerobot-train --dataset.repo_id=local/kitbot_act_v1_clean --dataset.root=$HOME/.cache/huggingface/lerobot/local/kitbot_act_v1_clean \
  --dataset.episodes="$EPS" --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --steps=50000 --save_freq=5000 --output_dir=outputs/train/act_kitbot_v1 --job_name=act_kitbot_v1 \
  > train_act_kitbot_v1.log 2>&1 &
tail -f train_act_kitbot_v1.log          # 진행 보기 (Ctrl+C는 보기만 끔, 학습은 계속)

# ③ train loss · eval loss 그래프 (학습이 끝난 뒤, 또는 중간에 저장된 체크포인트까지)
python -m kitbot_exec.loss_curve --run outputs/train/act_kitbot_v1_check     # → 그 폴더의 loss_curve.png, loss_curve.csv
```

- 왜 이 설정인지(기본값 유지, 단계 수, GPU 메모리를 다 쓰지 않는 이유): S05.md D05.
- **약 1.25배 빠르게(16비트 bf16)**: 위 명령의 `lerobot-train`을 `accelerate launch --num_processes=1 --mixed_precision=bf16 $(which lerobot-train)`으로 바꾼다(나머지 옵션 그대로). 300단계 실측 한 단계 0.49 → 0.39초, 손실 흐름 같음. 50,000단계 약 7시간 → 약 5.5시간(추정). `--policy.use_amp=true`는 ACT에 효과 없음(S05 검증 이력 2026-11-12).
- **이어서 더 학습** (예: 5,000단계 모델 → 20,000단계, bf16, 약 1시간 40분). 같은 폴더에 이어서 저장되고, eval용으로 뺀 에피소드도 그대로 유지된다:
  ```bash
  nohup setsid accelerate launch --num_processes=1 --mixed_precision=bf16 $(which lerobot-train) \
    --config_path=outputs/train/act_kitbot_v1_check/checkpoints/005000/pretrained_model/train_config.json \
    --resume=true --steps=20000 --save_freq=5000 >> train_act_kitbot_v1_check.log 2>&1 < /dev/null &
  ```
  - `--steps`는 이어서 더할 수가 아니라 **최종 단계 수**. 2시간이면 bf16으로 약 18,000단계(0.39초/단계)까지.
  - `nohup setsid … &`: 터미널이나 VS Code를 닫아도 학습이 계속된다. 진행은 `tail -f train_act_kitbot_v1_check.log`.
  - **충전기를 꼭 꽂는다.** 배터리만으로는 10% 아래에서 절전으로 18배 느려지고, 다 닳으면 PC가 꺼져 학습이 끊긴다(2026-11-12). 확인: `cat /sys/class/power_supply/A*/online` → 1. 끊기면 마지막 체크포인트(`checkpoints/<단계>`)의 `train_config.json`으로 같은 명령을 다시.
- 이 PC 실측(2026-11-12, 300단계 시험): RTX 4050에서 **약 2.0단계/초**, GPU 메모리 3.73GB(6GB 중), 배치 8, 손실 14.4 → 3.0(300단계). 기본 100,000단계는 약 14시간이라 50,000단계로 줄였다. `--policy.use_amp=true`는 속도·손실이 같아 효과 없음.
- 한 바퀴(에포크) = 학습 41개 20,785프레임 / 8 ≈ 2,600단계 → 50,000단계 ≈ 19바퀴.
- **eval loss**: lerobot-train은 실제 로봇 데이터의 eval loss를 계산하지 않는다(eval은 시뮬레이터 전용). 그래서 5개 에피소드를 학습에서 빼고, ③ `loss_curve`가 체크포인트마다 train·eval 에피소드에서 따로 계산한다.
  - `loss` = 학습 로그와 같은 식(L1 + KL × 10). `l1_run` = 실행 때와 같은 조건(잠재값 0)의 동작 오차. 둘 다 정규화한 관절값 기준.
  - eval이 train보다 크게 벌어지며 오르기 시작하면 과적합. 그 앞 체크포인트를 실물로 먼저 시험한다.
  - 5개를 뺀 만큼 데이터가 줄어든다. 좋은 단계 수를 정한 뒤 46개 전부로 다시 학습할 수 있다(`--dataset.episodes` 빼기).
- 체크포인트 하나는 약 0.6GB(최적화 상태 포함). 50,000단계를 5,000마다 저장하면 약 6GB.
- eval loss가 낮은 체크포인트 2~3개를 run_act로 같은 자리에서 실행해 비교한다(loss가 낮아도 실물 성공률이 꼭 높지는 않음). 예: `--policy outputs/train/act_kitbot_v1/checkpoints/020000/pretrained_model`.
- 학습 중 run_act를 같이 돌리면 GPU를 나눠 써서 둘 다 느려진다(실측: 앙상블 한 프레임 61ms → 약 16Hz, 목표 30Hz). 실물 시험 동안은 `python -m kitbot_exec.train_ctl pause`로 학습을 멈추고 끝나면 `resume` (2-6).
- 끊겼을 때 이어서: 아래 "끊긴 학습 이어서 하기"에서 경로를 `outputs/train/act_kitbot_v1`로.
- 실행은 2-6 `run_act.py --policy outputs/train/<job>/checkpoints/last/pretrained_model`.
- `--output_dir`이 이미 있으면 학습이 거부된다. 다시 하려면 다른 이름을 쓴다.

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
- **이 과제(S05)의 정책은 이 명령 대신 2-6 `run_act.py`로 실행한다.** 이 명령에는 CV 접근 단계가 없고 카메라 색 보정이 없어 녹화 때 영상과 달라진다.
