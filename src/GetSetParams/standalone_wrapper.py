#!/usr/bin/env python3

import subprocess
from pathlib import Path
import yaml

# --- Paths ---
LOG_DIR = Path.home() / "schaeffler" / "logs"
WORKSPACE_ROOT = LOG_DIR.parent             # ~/schaeffler
THIS_DIR = Path(__file__).resolve().parent  # ~/schaeffler/src/GetSetParams

PARAM_TEMPLATE = THIS_DIR / "nav2_params.yaml"   # base template
PARAM_OVERRIDE = THIS_DIR / "nav2_params_bo.yaml"  # file we generate per run


def write_params(max_vel_x: float, acc_lim_x: float):
    """
    Load nav2_params.yaml, override just max_vel_x and acc_lim_x,
    and write nav2_params_bo.yaml.
    """
    data = yaml.safe_load(PARAM_TEMPLATE.read_text())

    # Adjust ONLY the parameters you care about
    # Path found from your nav2_params.yaml:
    # controller_server -> ros__parameters -> FollowPath -> {max_vel_x, acc_lim_x}
    ctrl = data["controller_server"]["ros__parameters"]["FollowPath"]
    ctrl["max_vel_x"] = float(max_vel_x)
    ctrl["acc_lim_x"] = float(acc_lim_x)

    PARAM_OVERRIDE.write_text(yaml.safe_dump(data))
    print(
        f"[BO] Wrote params to {PARAM_OVERRIDE} "
        f"(max_vel_x={max_vel_x}, acc_lim_x={acc_lim_x})",
        flush=True,
    )

def load_latest_J():
    j_files = list(LOG_DIR.glob("J_nav2_run_*.txt"))
    if not j_files:
        raise RuntimeError(f"No J files found in {LOG_DIR}")

    latest = max(j_files, key=lambda p: p.stat().st_mtime)
    j_val = float(latest.read_text().strip())
    print(f"[BO] Latest J file: {latest.name}, J = {j_val}", flush=True)
    return j_val, latest


def run_one_sim(max_vel_x: float, acc_lim_x: float):
    """
    1) Write new params into nav2_params_bo.yaml
    2) Run the launch once with params_file:= that file
    3) Return latest J
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

    return load_latest_J()  # (J, path)


def main():
    # 🔹 For now: call it ONCE with some test values
    # Later BO will provide these numbers.
    test_max_vel_x = 0.31   # e.g. your current default
    test_acc_lim_x = 2.12

    J, path = run_one_sim(test_max_vel_x, test_acc_lim_x)
    print(f"[BO] Loaded J = {J} from {path}", flush=True)


if __name__ == "__main__":
    main()
