"""돌고 있는 lerobot-train을 잠깐 멈추고(GPU 비우기) 다시 돌린다. 실물 시험(run_act)을 학습 중에 할 때.

    cd project/src
    python -m kitbot_exec.train_ctl status     # 학습 프로세스·상태
    python -m kitbot_exec.train_ctl pause      # 일시정지 (GPU 계산을 멈춤. 메모리는 그대로 잡고 있음)
    python -m kitbot_exec.train_ctl resume     # 다시

멈춰 있는 동안 학습 결과는 바뀌지 않고, 끝나는 시각만 그만큼 늦어진다.
멈춘 채로 두면 학습이 끝나지 않으니 시험 뒤 꼭 resume.
"""
from __future__ import annotations

import argparse
import os
import signal
from pathlib import Path


def _cmdline(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
    except OSError:
        return ""


def _state(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except (OSError, IndexError):
        return "?"


def _ancestors(pid: int) -> set[int]:
    out = set()
    while pid > 1:
        out.add(pid)
        try:
            pid = int(Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return out


def train_pids() -> list[int]:
    """lerobot-train 본체와 그 자식(데이터 읽기 작업자). 이 명령 자신과 부모 셸은 뺀다."""
    me = _ancestors(os.getpid())
    pids = []
    for d in Path("/proc").iterdir():
        if d.name.isdigit():
            pid = int(d.name)
            if pid not in me and "lerobot-train" in _cmdline(pid) and "train_ctl" not in _cmdline(pid):
                pids.append(pid)
    return sorted(pids)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["status", "pause", "resume"])
    args = ap.parse_args(argv)
    pids = train_pids()
    if not pids:
        print("돌고 있는 lerobot-train 없음")
        return
    if args.action != "status":
        sig = signal.SIGSTOP if args.action == "pause" else signal.SIGCONT
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
    import time

    want = {"pause": len(pids), "resume": 0}.get(args.action)
    for _ in range(20):                        # 신호가 모든 프로세스에 닿을 때까지 잠깐 기다림
        stopped = sum(_state(pid) == "T" for pid in pids)
        if want is None or stopped == want:
            break
        time.sleep(0.1)
    print(f"학습 프로세스 {len(pids)}개: " + ("일시정지됨" if stopped == len(pids) else
                                         "돌고 있음" if stopped == 0 else f"{stopped}개만 멈춤 (다시 실행해 볼 것)"))


if __name__ == "__main__":
    main()
