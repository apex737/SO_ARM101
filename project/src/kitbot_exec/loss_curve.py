"""학습 loss(train)와 학습에 안 쓴 에피소드 loss(eval)를 체크포인트마다 계산해 그래프로 (S05-3).

lerobot-train은 실제 로봇 데이터에 대해 eval loss를 계산하지 않는다(eval은 시뮬레이터 전용).
그래서 학습 때 몇 에피소드를 빼 두고(--dataset.episodes), 저장된 체크포인트마다 여기서 따로 계산한다.

    cd project/src
    python -m kitbot_exec.loss_curve --run outputs/train/act_kitbot_v1_check

계산하는 값 (체크포인트마다, train 에피소드 일부와 eval 에피소드 전체에서)
  loss    학습 로그와 같은 식 = L1 + KL × 10 (VAE가 정답 동작을 보고 잠재값을 만듦, 드롭아웃 켜짐)
  l1_run  실행(run_act) 때와 같은 조건의 동작 오차 = 잠재값 0, 드롭아웃 꺼짐. 실물 성능에 더 가까운 값
  (두 값 모두 정규화한 관절값 기준. 1.0 ≈ 관절별 표준편차 1배 차이)
train과 eval 차이가 벌어지기 시작하면 과적합(학습 데이터만 외움) 신호다.

출력: <run>/loss_curve.png, <run>/loss_curve.csv
"""
from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

import numpy as np

DATASET_REPO = "local/kitbot_act_v1_clean"
# 학습에서 뺀 에피소드 (S05 D05): 보라 2·주황 2·파랑 1, 흔듦 2·안 흔듦 3, 위치 고르게
VAL_EPISODES = [6, 13, 22, 27, 36]


def parse_train_log(path: Path) -> list[tuple[int, float]]:
    """lerobot-train 출력에서 (단계, loss). 'step:1K' 같은 줄임 표기는 로그 순서로 보정한다."""
    out = []
    for line in path.read_text(errors="ignore").replace("\r", "\n").splitlines():
        m = re.search(r"step:(\d+(?:\.\d+)?)([KM]?) .*?loss:([\d.]+)", line)
        if m:
            v = float(m.group(1)) * {"": 1, "K": 1e3, "M": 1e6}[m.group(2)]
            out.append((int(v), float(m.group(3))))
    # 1K, 2K처럼 반올림된 단계는 log_freq 간격으로 다시 매긴다
    if len(out) >= 2:
        freq = out[0][0] if out[0][0] > 0 else out[1][0] - out[0][0]
        out = [(freq * (i + 1), l) for i, (_, l) in enumerate(out)]
    return out


def checkpoints(run: Path) -> list[tuple[int, Path]]:
    cks = []
    for d in (run / "checkpoints").glob("[0-9]*"):
        pm = d / "pretrained_model"
        if pm.is_dir():
            cks.append((int(d.name), pm))
    return sorted(cks)


def frame_indices(ds, stride: int, max_frames: int | None, seed: int = 0) -> list[int]:
    idx = list(range(0, len(ds), stride))
    if max_frames and len(idx) > max_frames:
        idx = sorted(np.random.default_rng(seed).choice(idx, max_frames, replace=False).tolist())
    return idx


def eval_checkpoint(pm: Path, datasets: dict, device: str, batch_size: int = 8, seed: int = 0) -> dict:
    """{이름: {loss, l1_vae, kld, l1_run}}"""
    import torch
    from torch.utils.data import DataLoader, Subset

    from .run_act import load_policy

    policy, pre, _post, dev = load_policy(str(pm), device)
    res = {}
    for name, (ds, idx) in datasets.items():
        dl = DataLoader(Subset(ds, idx), batch_size=batch_size, shuffle=False, num_workers=4)
        sums = {"loss": 0.0, "l1_vae": 0.0, "kld": 0.0, "l1_run": 0.0}
        n = 0
        torch.manual_seed(seed)
        with torch.no_grad():
            for batch in dl:
                b = pre(batch)
                policy.train()                       # 학습 때와 같은 식 (VAE + 드롭아웃)
                loss, d = policy.forward(b)
                policy.eval()                        # 실행 때와 같은 조건 (잠재값 0)
                _, d_run = policy.forward(b)
                k = len(batch["index"])
                sums["loss"] += float(loss) * k
                sums["l1_vae"] += d["l1_loss"] * k
                sums["kld"] += d.get("kld_loss", 0.0) * k
                sums["l1_run"] += d_run["l1_loss"] * k
                n += k
        res[name] = {key: v / max(n, 1) for key, v in sums.items()} | {"frames": n}
    del policy
    torch.cuda.empty_cache()
    return res


def plot(rows: list[dict], log_curve: list[tuple[int, float]], out: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    have = {f.name for f in font_manager.fontManager.ttflist}
    ko = [f for f in ("NanumSquare", "NanumGothic", "Noto Sans CJK KR", "Noto Serif CJK KR") if f in have]
    plt.rcParams["font.family"] = ko[:1] + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    ax = axes[0]
    if log_curve:
        s, l = zip(*log_curve)
        ax.plot(s, l, color="#bbbbbb", lw=1, label="train loss (학습 로그)")
    st = [r["step"] for r in rows]
    ax.plot(st, [r["train_loss"] for r in rows], "o-", color="#1f77b4", label="train loss (체크포인트)")
    ax.plot(st, [r["eval_loss"] for r in rows], "s-", color="#d62728", label="eval loss (학습에 안 쓴 에피소드)")
    ax.set_yscale("log"); ax.set_xlabel("step"); ax.set_title("loss = L1 + 10·KL (학습 식)"); ax.grid(alpha=.3)
    ax.legend()
    ax = axes[1]
    ax.plot(st, [r["train_l1_run"] for r in rows], "o-", color="#1f77b4", label="train")
    ax.plot(st, [r["eval_l1_run"] for r in rows], "s-", color="#d62728", label="eval")
    ax.set_yscale("log"); ax.set_xlabel("step"); ax.set_title("실행 조건 동작 오차 L1 (잠재값 0)"); ax.grid(alpha=.3)
    ax.legend()
    from matplotlib.ticker import FuncFormatter, LogLocator
    for ax in axes:   # 로그 축 눈금을 0.25 같은 보통 숫자로 (지수 표기는 한글 글꼴에서 깨짐)
        ax.yaxis.set_major_locator(LogLocator(subs=(1.0, 2.0, 5.0)))
        ax.yaxis.set_minor_locator(LogLocator(subs=(1.5, 3.0, 4.0, 7.0)))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
        ax.yaxis.set_minor_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="lerobot-train --output_dir 폴더")
    ap.add_argument("--repo-id", default=DATASET_REPO)
    ap.add_argument("--root", help="데이터셋 폴더 (기본 ~/.cache/huggingface/lerobot/<repo-id>)")
    ap.add_argument("--val", default=",".join(map(str, VAL_EPISODES)), help="eval 에피소드 번호 (쉼표)")
    ap.add_argument("--stride", type=int, default=5, help="몇 프레임마다 하나씩 계산할지")
    ap.add_argument("--max-train-frames", type=int, default=1500, help="train 쪽은 이만큼만 무작위로 (시간 절약)")
    ap.add_argument("--log", help="학습 로그 파일 (train loss 선). 기본: project/src/train_<job>.log")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--plot-only", action="store_true", help="계산하지 않고 저장된 loss_curve.csv로 그래프만 다시")
    args = ap.parse_args(argv)

    import lerobot.policies  # noqa: F401  (ACT 등 정책 설정 종류를 등록해야 설정 파일을 읽을 수 있음)
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets import LeRobotDataset, LeRobotDatasetMetadata
    from lerobot.datasets.factory import resolve_delta_timestamps

    from .record_act import dataset_root

    run = Path(args.run)
    log = Path(args.log) if args.log else Path(f"train_{run.name}.log")
    if args.plot_only:
        with (run / "loss_curve.csv").open() as f:
            rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
        plot(rows, parse_train_log(log) if log.exists() else [], run / "loss_curve.png", f"{run.name}")
        print(f"그래프: {run / 'loss_curve.png'}")
        return
    cks = checkpoints(run)
    if not cks:
        raise SystemExit(f"체크포인트 없음: {run}/checkpoints/*/pretrained_model")
    root = str(dataset_root(args.repo_id, args.root))
    meta = LeRobotDatasetMetadata(args.repo_id, root=root)
    val = [int(v) for v in args.val.split(",") if v.strip()]
    train = [e for e in range(meta.total_episodes) if e not in val]
    cfg = PreTrainedConfig.from_pretrained(str(cks[0][1]))
    dts = resolve_delta_timestamps(cfg, meta)
    ds_tr = LeRobotDataset(args.repo_id, root=root, episodes=train, delta_timestamps=dts)
    ds_va = LeRobotDataset(args.repo_id, root=root, episodes=val, delta_timestamps=dts)
    sets = {"train": (ds_tr, frame_indices(ds_tr, args.stride, args.max_train_frames)),
            "eval": (ds_va, frame_indices(ds_va, args.stride, None))}
    print(f"train 에피소드 {len(train)}개 중 {len(sets['train'][1])}프레임, eval 에피소드 {val} 중 {len(sets['eval'][1])}프레임")

    rows = []
    for step, pm in cks:
        r = eval_checkpoint(pm, sets, args.device)
        row = {"step": step}
        for name in ("train", "eval"):
            for k, v in r[name].items():
                row[f"{name}_{k}"] = round(v, 5) if isinstance(v, float) else v
        rows.append(row)
        print(f"  {step:>7}단계  train loss {row['train_loss']:.4f} (l1_run {row['train_l1_run']:.4f})   "
              f"eval loss {row['eval_loss']:.4f} (l1_run {row['eval_l1_run']:.4f})   eval/train l1_run {row['eval_l1_run'] / max(row['train_l1_run'], 1e-9):.2f}배")

    with (run / "loss_curve.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)
    job = run.name
    log_curve = parse_train_log(log) if log.exists() else []
    plot(rows, log_curve, run / "loss_curve.png", f"{job}: train {len(train)} ep / eval {len(val)} ep")
    print(f"그래프: {run / 'loss_curve.png'}\n표: {run / 'loss_curve.csv'}")


if __name__ == "__main__":
    main()
