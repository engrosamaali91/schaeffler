#!/usr/bin/env python3

import subprocess
from pathlib import Path
import yaml
import csv

# ---------------- Paths ----------------

LOG_DIR = Path.home() / "schaeffler" / "logs"
WORKSPACE_ROOT = LOG_DIR.parent             # ~/schaeffler
THIS_DIR = Path(__file__).resolve().parent  # ~/schaeffler/src/GetSetParams

PARAM_TEMPLATE = THIS_DIR / "nav2_params.yaml"       # base template
PARAM_OVERRIDE = THIS_DIR / "nav2_params_bo.yaml"    # file we generate
BO_EVALS_CSV = THIS_DIR / "bo_evals.csv"             # evals logged HERE


# ---------------- Helpers ----------------

def write_params(max_vel_x: float, acc_lim_x: float):
    """
    Load nav2_params.yaml, override just max_vel_x and acc_lim_x in FollowPath,
    and write nav2_params_bo.yaml.
    """
    data = yaml.safe_load(PARAM_TEMPLATE.read_text())

    # controller_server -> ros__parameters -> FollowPath
    ctrl = data["controller_server"]["ros__parameters"]["FollowPath"]
    ctrl["max_vel_x"] = float(max_vel_x)
    ctrl["max_speed_xy"] = float(max_vel_x)   # keep consistent if you use this param
    ctrl["acc_lim_x"] = float(acc_lim_x)

    PARAM_OVERRIDE.write_text(yaml.safe_dump(data))
    print(
        f"[BO] Wrote params to {PARAM_OVERRIDE} "
        f"(max_vel_x={max_vel_x:.4f}, acc_lim_x={acc_lim_x:.4f})",
        flush=True,
    )


def load_latest_J():
    j_files = list(LOG_DIR.glob("J_nav2_run_*.txt"))
    if not j_files:
        raise RuntimeError(f"No J files found in {LOG_DIR}")

    latest = max(j_files, key=lambda p: p.stat().st_mtime)
    j_val = float(latest.read_text().strip())
    print(f"[BO] Latest J file: {latest.name}, J = {j_val:.5f}", flush=True)
    return j_val, latest


def run_one_sim(max_vel_x: float, acc_lim_x: float) -> float:
    """
    1) Write new params into nav2_params_bo.yaml
    2) Run the launch once with params_file:= that file
    3) Return latest J

    NOTE: With current launch, you still press Ctrl+C once when you're happy
    that the run + KPI are finished. That completely shuts down Isaac+Nav2.
    """
    write_params(max_vel_x, acc_lim_x)

    cmd = [
        "ros2", "launch",
        "agv_orchestrator", "isaac_and_nav2.launch.py",
        "rviz:=true",
        "run_test:=true",
        "compute_kpi:=true",
        f"params_file:={PARAM_OVERRIDE.absolute()}",
    ]

    try:
        subprocess.run(cmd, check=True, cwd=str(WORKSPACE_ROOT))
    except KeyboardInterrupt:
        print("\n[BO] Ctrl+C detected, assuming run finished. Reading latest J...", flush=True)

    J, _ = load_latest_J()
    return J


def init_csv_if_needed():
    """Create bo_evals.csv with header if it does not exist."""
    if not BO_EVALS_CSV.exists():
        with BO_EVALS_CSV.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["iter", "max_vel_x", "acc_lim_x", "J"])
        print(f"[BO] Created {BO_EVALS_CSV} with header.", flush=True)


def append_eval(iter_idx: int, max_vel_x: float, acc_lim_x: float, J: float):
    """Append one row to bo_evals.csv in the CURRENT directory."""
    with BO_EVALS_CSV.open("a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([iter_idx, f"{max_vel_x:.6f}", f"{acc_lim_x:.6f}", f"{J:.6f}"])
    print(f"[BO] Logged iter={iter_idx}, J={J:.5f} to {BO_EVALS_CSV}", flush=True)


# ---------------- Two-iteration test loop ----------------

def main():
    init_csv_if_needed()

    # For now: explicit 2 candidates (you can choose any values you want)
    param_candidates = [
        (0.26, 2.5),  # iteration 1
        (0.30, 2.5),  # iteration 2 (different max_vel_x)
    ]

    for it, (max_vel_x, acc_lim_x) in enumerate(param_candidates, start=1):
        print("\n" + "=" * 60)
        print(f"[BO] Iteration {it}/2")
        print("=" * 60, flush=True)

        J = run_one_sim(max_vel_x, acc_lim_x)
        print(f"[BO] Iter {it} got J = {J:.5f}", flush=True)

        append_eval(it, max_vel_x, acc_lim_x, J)

    print("\n" + "#" * 60)
    print("[BO] Two-run test finished.")
    print("#" * 60)


if __name__ == "__main__":
    main()
