"""Kaggle에서 학습한 디퓨전 정책(20,000단계)을 실물에서 실행한다. run_act.py와 같은 흐름·판정·기록, 기본값만 디퓨전에 맞춤.

    cd project/src
    python -m kitbot_exec.run_diffusion --color purple --trials 3 --step     # 처음
    python -m kitbot_exec.run_diffusion --color purple                       # 18회, 기록 var/compare/diffusion_20k.jsonl
    python -m kitbot_exec.compare var/compare/ik_v2.jsonl var/compare/act_20k.jsonl var/compare/diffusion_20k.jsonl

모델: outputs/train/diffusion_kaggle_20k/checkpoint/pretrained_model
  (~/Downloads/diffusion_latest_020000_*.zip을 푼 것. 학습 상태 2.2GB는 풀지 않음)
  데이터 apex737/kitbot_act_v1 50개 전부(걸러 내기 전, 10%는 eval), 관찰 2프레임 → 16프레임 예측 → 8프레임 실행,
  영상은 240×320으로 줄여 입력(정책 안에서), DDIM 10단계.

ACT와 다른 점 (이 PC RTX 4050 실측)
  - 8프레임마다 새로 예측하는데 한 번에 약 0.26초 걸려, 그동안 팔이 멈춘다 → 움직임이 뚝뚝 끊기고 전체가 약 2배 느리다.
    그래서 시간 제한 기본값을 60초로 둔다. 끝에 "늦은 프레임" 경고가 나오는 것은 정상.
  - --n-action-steps 15: 한 번 예측으로 15프레임 실행(멈춤 횟수 절반, 대신 그동안 새 관찰을 안 봄). 학습 때는 8.
  - --num-inference-steps 5: 예측 시간 절반(약 0.13초), 대신 동작이 달라진다(무작위 입력에서 평균 2°, 최대 14° 차이).
  - --temporal-ensemble은 ACT 전용이라 쓸 수 없다.
"""
from __future__ import annotations

import sys

from . import run_act

POLICY = "outputs/train/diffusion_kaggle_20k/checkpoint/pretrained_model"
DEFAULTS = {"--policy": POLICY, "--name": "diffusion_20k", "--max-s": "60"}
MAX_N_ACTION = 15      # horizon 16 − n_obs 2 + 1


def build_argv(argv: list[str]) -> list[str]:
    out = list(argv)
    for k, v in DEFAULTS.items():
        if k not in out:
            out += [k, v]
    if "--n-action-steps" in out:
        n = int(out[out.index("--n-action-steps") + 1])
        if not 1 <= n <= MAX_N_ACTION:
            raise SystemExit(f"--n-action-steps는 1~{MAX_N_ACTION} (예측 16프레임 중 관찰 시각부터 쓸 수 있는 수)")
    return out


def main(argv=None):
    run_act.main(build_argv(sys.argv[1:] if argv is None else argv))


if __name__ == "__main__":
    main()
