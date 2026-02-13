#!/usr/bin/env python3

import subprocess
from pathlib import Path
import yaml
import csv
import time
from bayes_opt import BayesianOptimization

# ---------------- Paths ----------------

LOG_DIR = Path.home() / "schaeffler" / "logs"
WORKSPACE_ROOT = LOG_DIR.parent             # ~/schaeffler
THIS_DIR = Path(__file__).resolve().parent  # ~/schaeffler/src/GetSetParams

PARAM_TEMPLATE = THIS_DIR / "nav2_params_modified.yaml"       # base template
PARAM_OVERRIDE = THIS_DIR / "nav2_params_bo.yaml"    # file we generate
BO_EVALS_CSV = THIS_DIR / "bo_evals.csv"             # evals logged HERE


# ---------------- Search bounds ----------------
# Updated ranges based on your manual tuning results.
# We focus on the low-acceleration region to capture the "inertia".
pbounds = {
    "max_vel_x":     (0.12, 0.22),  # Centered around your good 0.16
    "acc_lim_x":     (0.02, 0.22),  # Centered around your good 0.05
    "max_vel_theta": (0.10, 0.50),  # Centered around your good 0.2
    "acc_lim_theta": (0.03, 0.50),  # Centered around your good 0.1
}


# ---------------- Helpers ----------------

def write_params(max_vel_x: float, acc_lim_x: float, max_vel_theta: float, acc_lim_theta: float):
    """
    Load nav2_params.yaml, override linear AND angular params,
    and write nav2_params_bo.yaml.
    """
    # 1. Read Template
    if not PARAM_TEMPLATE.exists():
        raise FileNotFoundError(f"Template not found at {PARAM_TEMPLATE}")
        
    data = yaml.safe_load(PARAM_TEMPLATE.read_text())

    # 2. Update Controller Server (DWB)
    # controller_server -> ros__parameters -> FollowPath
    ctrl = data["controller_server"]["ros__parameters"]["FollowPath"]
    
    # Linear (X) - Rounded to 4 decimal places
    ctrl["max_vel_x"] = round(float(max_vel_x), 3)
    ctrl["max_speed_xy"] = round(float(max_vel_x), 3)
    ctrl["acc_lim_x"] = round(float(acc_lim_x), 3)
    ctrl["decel_lim_x"] = -round(float(acc_lim_x), 3) # Symmetric decel

    # Angular (Theta) - Rounded to 4 decimal places
    ctrl["max_vel_theta"] = round(float(max_vel_theta), 3)
    ctrl["acc_lim_theta"] = round(float(acc_lim_theta), 3)
    ctrl["decel_lim_theta"] = -round(float(acc_lim_theta), 3) # Symmetric decel

    # 3. Update Velocity Smoother (to match limits)
    # velocity_smoother -> ros__parameters
    vs = data["velocity_smoother"]["ros__parameters"]
    
    # Lists are [x, y, theta]
    # Update X (index 0)
    vs["max_velocity"][0] = round(float(max_vel_x), 4)
    vs["min_velocity"][0] = -round(float(max_vel_x), 4)
    vs["max_accel"][0]    = round(float(acc_lim_x), 4)
    vs["max_decel"][0]    = -round(float(acc_lim_x), 4)

    # Update Theta (index 2)
    vs["max_velocity"][2] = round(float(max_vel_theta), 4)
    vs["min_velocity"][2] = -round(float(max_vel_theta), 4)
    vs["max_accel"][2]    = round(float(acc_lim_theta), 4)
    vs["max_decel"][2]    = -round(float(acc_lim_theta), 4)

    # 4. Write to file
    PARAM_OVERRIDE.write_text(yaml.safe_dump(data))
    print(
        f"[BO] Wrote params: "
        f"vel_x={ctrl['max_vel_x']}, acc_x={ctrl['acc_lim_x']}, "
        f"vel_th={ctrl['max_vel_theta']}, acc_th={ctrl['acc_lim_theta']}",
        flush=True,
    )


def load_latest_J(timeout=10, check_interval=1):
    """
    Load latest J value. Ensures we don't read a stale file.
    """
    start_time = time.time()
    
    # 1. Find the timestamp of the nav2 CSV we just generated
    latest_nav2_time = 0
    nav2_files = list(LOG_DIR.glob("nav2_run_*.csv"))
    if nav2_files:
        latest_nav2 = max(nav2_files, key=lambda p: p.stat().st_mtime)
        latest_nav2_time = latest_nav2.stat().st_mtime
    
    # 2. Wait for a J file strictly NEWER than that CSV
    while time.time() - start_time < timeout:
        j_files = list(LOG_DIR.glob("J_nav2_run_*.txt"))
        if j_files:
            latest_j_file = max(j_files, key=lambda p: p.stat().st_mtime)
            j_time = latest_j_file.stat().st_mtime
            
            # Simple check: is J file newer than the nav2 data file?
            if j_time > latest_nav2_time:
                try:
                    val = float(latest_j_file.read_text().strip())
                    print(f"[BO] Found fresh J file: {latest_j_file.name}, J={val:.5f}", flush=True)
                    return val
                except ValueError:
                    pass # File might be empty/writing
            
        time.sleep(check_interval)
    
    # Fallback/Debug if something hangs
    print("[BO] WARNING: Timeout waiting for fresh J file. Returning penalty J=10.0", flush=True)
    return 10.0 


def run_one_sim(max_vel_x, acc_lim_x, max_vel_theta, acc_lim_theta) -> float:
    # 1. Write Params
    write_params(max_vel_x, acc_lim_x, max_vel_theta, acc_lim_theta)

    # 2. Launch
    cmd = [
        "ros2", "launch",
        "agv_orchestrator", "isaac_and_nav2.launch.py",
        "rviz:=true",
        "run_test:=true",
        "compute_kpi:=true",
        f"params_file:={PARAM_OVERRIDE.absolute()}",
    ]

    print("[BO] Launching Sim...", flush=True)
    try:
        # We wait until the process exits (Ctrl+C manually or script auto-close)
        subprocess.run(cmd, check=True, cwd=str(WORKSPACE_ROOT))
    except KeyboardInterrupt:
        print("\n[BO] Ctrl+C detected. Checking for results...", flush=True)

    # 3. Read Result
    J = load_latest_J()
    return J


def init_csv_if_needed():
    if not BO_EVALS_CSV.exists():
        with BO_EVALS_CSV.open("w", newline="") as f:
            writer = csv.writer(f)
            # Added new columns
            writer.writerow(["iter", "max_vel_x", "acc_lim_x", "max_vel_theta", "acc_lim_theta", "J"])
        print(f"[BO] Created {BO_EVALS_CSV} with header.", flush=True)


def append_eval(iter_idx, max_vel_x, acc_lim_x, max_vel_theta, acc_lim_theta, J):
    with BO_EVALS_CSV.open("a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            iter_idx, 
            f"{max_vel_x:.6f}", f"{acc_lim_x:.6f}", 
            f"{max_vel_theta:.6f}", f"{acc_lim_theta:.6f}", 
            f"{J:.6f}"
        ])


# ---------------- BO objective ----------------

_iter_counter = {"i": 0}

def objective(max_vel_x, acc_lim_x, max_vel_theta, acc_lim_theta):
    _iter_counter["i"] += 1
    it = _iter_counter["i"]

    print("\n" + "=" * 60)
    print(f"[BO] Iteration {it}")
    print(f"[BO] Testing: lin_vel={max_vel_x:.3f}, lin_acc={acc_lim_x:.3f}, ang_vel={max_vel_theta:.3f}, ang_acc={acc_lim_theta:.3f}")
    print("=" * 60, flush=True)

    J = run_one_sim(max_vel_x, acc_lim_x, max_vel_theta, acc_lim_theta)
    
    append_eval(it, max_vel_x, acc_lim_x, max_vel_theta, acc_lim_theta, J)

    # Minimize J <=> Maximize -J
    return -J


# ---------------- Main ----------------

def main():
    init_csv_if_needed()

    optimizer = BayesianOptimization(
        f=objective,
        pbounds=pbounds,
        verbose=2,
        random_state=42, # Set seed for reproducibility
    )

    print(f"[BO] Starting optimization with bounds: {pbounds}")
    
    # Suggest the "known good" manual point first to jumpstart BO
    # (Optional, but helps it converge faster)
    # optimizer.probe(
    #     params={
    #         "max_vel_x": 0.16, 
    #         "acc_lim_x": 0.05, 
    #         "max_vel_theta": 0.2, 
    #         "acc_lim_theta": 0.1
    #     },
    #     lazy=True,
    # )

    optimizer.maximize(
        init_points=2,   # Random exploration steps
        n_iter=10,       # Optimization steps
    )

    print("\n" + "#" * 60)
    print(f"[BO] Best result found:")
    print(optimizer.max)
    print("#" * 60)


if __name__ == "__main__":
    main()