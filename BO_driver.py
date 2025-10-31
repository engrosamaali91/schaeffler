#!/usr/bin/env python3
# bo_autotune.py
import os, json, yaml, shlex, time, signal
import subprocess as sp
from pathlib import Path
from datetime import datetime
from bayes_opt import BayesianOptimization

# --------- CONFIG (edit these to your paths) ----------
PROJECT = Path.home() / "schaeffler"
PKG     = PROJECT / "src" / "GetSetParams"      # your scripts live here
RUNS    = PROJECT / "runs_bo"                   # logs per trial

BASE_NAV2 = PROJECT / "install" / "nav_bringup" / "share" / "nav_bringup" / "config" / "nav2_params.yaml"
BOUNDS_YAML = PKG / "bounds.yaml"               # per-key bounds
BAG_TO_CSV  = PKG / "bag_to_csv.py"             # converts /odom bag -> sim_odom.csv
COMPUTE_J   = PKG / "compute_j.py"              # prints numeric J to stdout
REF_CSV     = PKG / "groundtruth.csv"           # real robot reference

LAUNCH_PKG  = "agv_orchestrator"
LAUNCH_FILE = "isaac_and_nav2.launch.py"

# keys you are tuning (param names in BO) -> dotted YAML keys in Nav2
KEY_MAP = {
    "max_vel_x":  "controller_server.FollowPath.max_vel_x",
    "acc_lim_x":  "controller_server.FollowPath.acc_lim_x",
    # add more later, e.g.:
    # "max_vel_theta":  "controller_server.FollowPath.max_vel_theta",
    # "acc_lim_theta":  "controller_server.FollowPath.acc_lim_theta",
}

# optional: kill the launch if it exceeds this many seconds
LAUNCH_TIMEOUT = 600

# --------- helpers ----------
def run(cmd, check=True, capture=False, timeout=None, popen=False):
    print(">>", " ".join(shlex.quote(str(c)) for c in cmd))
    if popen:
        return sp.Popen(cmd, stdout=sp.PIPE, stderr=sp.STDOUT, text=True)
    if capture:
        return sp.run(cmd, check=check, text=True, capture_output=True, timeout=timeout)
    return sp.run(cmd, check=check, timeout=timeout)

def set_nested(d, dotted, value):
    node, rest = dotted.split(".", 1)
    cur = d.setdefault(node, {}).setdefault("ros__parameters", {})
    parts = rest.split(".")
    for p in parts[:-1]:
        cur = cur.setdefault(p, {})
    cur[parts[-1]] = value

def write_trial_yaml(base_yaml, out_yaml, params):
    with open(base_yaml, "r") as f:
        doc = yaml.safe_load(f) or {}
    for p_name, p_val in params.items():
        set_nested(doc, KEY_MAP[p_name], p_val)
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    with open(out_yaml, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)

def load_pbounds(bounds_yaml):
    with open(bounds_yaml, "r") as f:
        b = yaml.safe_load(f) or {}
    pb = {}
    for p_name, dotted in KEY_MAP.items():
        if dotted not in b:
            raise ValueError(f"bounds.yaml missing entry for {dotted}")
        lo, hi = b[dotted]
        pb[p_name] = (float(lo), float(hi))
    return pb

# --------- trial execution: launch → bag → csv → J ----------
trial_idx = {"i": 0}  # mutable counter captured by evaluate()

def run_trial_and_compute_J(params):
    i = trial_idx["i"]; trial_idx["i"] += 1
    tag = datetime.now().strftime(f"bo_%Y%m%d_%H%M%S_t{i:02d}")
    out_dir = RUNS / tag; out_dir.mkdir(parents=True, exist_ok=True)

    trial_yaml = out_dir / "nav2_trial.yaml"
    bag_dir    = out_dir / "bag"
    csv_path   = out_dir / "sim_odom.csv"

    # 1) Write param YAML for this BO suggestion
    write_trial_yaml(BASE_NAV2, trial_yaml, params)

    # 2) Start rosbag recorder (if your launch doesn’t record internally)
    bag_proc = run(["ros2", "bag", "record", "-o", str(bag_dir), "/odom"], popen=True)
    time.sleep(1.0)  # small delay to ensure recorder is ready

    # 3) Launch your stack (blocks until scenario ends / shutdown)
    try:
        run([
            "ros2", "launch", LAUNCH_PKG, LAUNCH_FILE,
            f"rviz:=false",
            f"run_test:=true",
            f"params_file:={trial_yaml}",
            # if you added recording args to your launch, you could pass:
            # f"record_bag:=true", f"bag_out:={bag_dir}",
        ], timeout=LAUNCH_TIMEOUT)
    finally:
        # stop bag recorder gracefully
        try:
            bag_proc.send_signal(signal.SIGINT)
            bag_proc.wait(timeout=15)
        except Exception:
            bag_proc.kill()

    # 4) Convert bag → CSV
    run(["python3", str(BAG_TO_CSV), "--in", str(bag_dir), "--out", str(csv_path)])

    # 5) Compute J (lower is better); expect numeric J on stdout
    res = run(["python3", str(COMPUTE_J), "--sim", str(csv_path), "--ref", str(REF_CSV)], capture=True)
    J = float(res.stdout.strip())
    print(f"[trial {i}] J = {J:.6f}  params={params}")
    return J

# --------- BO objective (BO maximizes) ----------
def make_objective():
    def evaluate(**kwargs):
        J = run_trial_and_compute_J(kwargs)
        return -J
    return evaluate

# --------- main  ----------
def main():
    RUNS.mkdir(parents=True, exist_ok=True)

    # pbounds from bounds.yaml (so BO == real robot limits)
    pbounds = load_pbounds(BOUNDS_YAML)

    optimizer = BayesianOptimization(
        f=make_objective(),
        pbounds=pbounds,
        random_state=123,
        allow_duplicate_points=True,
    )

    # warmup explorations
    INIT = 4
    N_ITER = 30
    TARGET_J = 1.0

    optimizer.maximize(init_points=INIT, n_iter=0)

    for k in range(N_ITER):
        optimizer.maximize(init_points=0, n_iter=1, acq="ei")
        best_J = -optimizer.max["target"]
        print(f"[BO iter {k+1}] best J = {best_J:.6f} at {optimizer.max['params']}")
        if best_J < TARGET_J:
            print("✅ Early stop: reached target J")
            break

    print("\n=== BO summary ===")
    print("best params:", optimizer.max["params"])
    print("best J     :", -optimizer.max["target"])

if __name__ == "__main__":
    main()
