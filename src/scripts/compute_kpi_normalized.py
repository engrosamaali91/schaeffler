#!/usr/bin/env python3
"""
compute_kpi_normalized.py

Usage examples:

  # Fully automatic (uses defaults in ~/schaeffler/logs)
  python compute_kpi_normalized.py

  # Only override sim log
  python compute_kpi_normalized.py --sim /some/path/nav2_run_42.csv

  # Override both explicitly (old behaviour)
  python compute_kpi_normalized.py --real real_log.csv --sim sim_log.csv \
      --pos-th 0.03 --psi-th 0.02

Expected CSV format (headers required):
  t,x,y,yaw[,vx]

- t    in seconds (monotonic, float)
- x,y  in meters (map/world frame)
- yaw  in radians (map/world frame)
- vx   optional, used only for plotting

Defaults for automatic mode (for your setup):
  real CSV:  ~/schaeffler/logs/omron_run.csv
  sim  CSV:  latest ~/schaeffler/logs/nav2_run_*.csv

Outputs:
  Prints RMSE_pos [m], RMSE_psi [rad], J_tilde to stdout and shows plots.
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
import math


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def add_cumdist(df):
    dx = np.diff(df["x"].to_numpy(), prepend=df["x"].iloc[0])
    dy = np.diff(df["y"].to_numpy(), prepend=df["y"].iloc[0])
    s  = np.cumsum(np.hypot(dx, dy))
    df = df.copy()
    df["s"] = s
    return df

def crop_to_distance(df, s_target):
    # keep samples until cumulative distance >= s_target
    idx = np.searchsorted(df["s"].to_numpy(), s_target, side="right") - 1
    idx = max(1, min(idx, len(df)-1))
    return df.iloc[:idx+1].reset_index(drop=True)

def angle_wrap_diff(a, b):
    d = a - b
    return np.arctan2(np.sin(d), np.cos(d))

def rmse(arr):
    return float(np.sqrt(np.mean(np.square(arr))))

def resample_to(common_t, df):
    """Resample df to common_t using linear interpolation.
    Resampling to 10 Hz ensures both datasets have the same time steps.
    """
    out = {}
    for col in df.columns:
        if col == "t":
            continue
        out[col] = np.interp(common_t, df["t"].to_numpy(), df[col].to_numpy())
    out_df = pd.DataFrame({"t": common_t, **out})
    return out_df

def normalize_to_start_frame(df: pd.DataFrame) -> pd.DataFrame:
    """Translate to (0,0) and rotate by -yaw0 so initial heading is +X.
    Normalization ensures both datasets are in the same reference frame.
    """
    df = df.copy()
    # ensure clean, numeric, monotonic time
    for c in ["t", "x", "y", "yaw"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t", "x", "y", "yaw"]).sort_values("t")
    df = df.loc[~df["t"].duplicated(keep="first")].reset_index(drop=True)
    if df.empty:
        return df

    x0 = float(df.loc[0, "x"])
    y0 = float(df.loc[0, "y"])
    psi0 = float(df.loc[0, "yaw"])

    # Calculate displacement
    dx = df["x"] - x0
    dy = df["y"] - y0
    c = np.cos(-psi0)
    s = np.sin(-psi0)

    # Rotate the data by -yaw0 (this aligns both datasets to the same frame)
    df["x"] = c * dx - s * dy
    df["y"] = s * dx + c * dy
    # make yaw relative to start (wrapped)
    df["yaw"] = angle_wrap_diff(df["yaw"].to_numpy(), psi0)
    return df

def compute_kpis(real_df, sim_df, T_pos, T_psi):
    # Overlapping time window (after normalization)
    t_min = max(real_df["t"].min(), sim_df["t"].min())
    t_max = min(real_df["t"].max(), sim_df["t"].max())
    if t_max <= t_min:
        raise ValueError("Time ranges do not overlap between real and sim.")

    # Resample to common grid
    hz = 10.0  # resample to real robot frequency
    dt = 1.0 / hz
    common_t = np.arange(t_min, t_max, dt)

    real_i = resample_to(common_t, real_df)
    sim_i  = resample_to(common_t, sim_df)

    # Position RMSE
    dx = sim_i["x"].to_numpy() - real_i["x"].to_numpy()
    dy = sim_i["y"].to_numpy() - real_i["y"].to_numpy()
    rmse_pos = rmse(np.sqrt(dx * dx + dy * dy))

    # Yaw RMSE (wrapped)
    dyaw = angle_wrap_diff(sim_i["yaw"].to_numpy(), real_i["yaw"].to_numpy())
    rmse_psi = rmse(dyaw)

    J_tilde = float(np.mean([rmse_pos / T_pos, rmse_psi / T_psi]))

    # Sanity check with delta yaw (Δyaw)
    print(f"[overlap] t ∈ [{common_t[0]:.2f}, {common_t[-1]:.2f}] s")
    print(
        f"  sim  Δx={sim_i['x'].iloc[-1]-sim_i['x'].iloc[0]:.3f}, "
        f"Δy={sim_i['y'].iloc[-1]-sim_i['y'].iloc[0]:.3f}, "
        f"Δyaw={angle_wrap_diff(sim_i['yaw'].iloc[-1], sim_i['yaw'].iloc[0]):.3f}"
    )
    print(
        f"  real Δx={real_i['x'].iloc[-1]-real_i['x'].iloc[0]:.3f}, "
        f"Δy={real_i['y'].iloc[-1]-real_i['y'].iloc[0]:.3f}, "
        f"Δyaw={angle_wrap_diff(real_i['yaw'].iloc[-1], real_i['yaw'].iloc[0]):.3f}"
    )

    return rmse_pos, rmse_psi, J_tilde, hz, len(common_t)

# def plot_trajectories(real_df, sim_df):
#     plt.figure(figsize=(12, 16))  # Adjusting the figure size to fit all 5 plots

#     # Plot the velocity in x (vx) over time (1st plot)
#     plt.subplot(5, 1, 1)
#     if "vx" in real_df.columns:
#         plt.plot(real_df["t"], real_df["vx"], label="Real vx")
#     if "vx" in sim_df.columns:
#         plt.plot(sim_df["t"], sim_df["vx"], label="Sim vx", linestyle="--")
#     plt.ylabel("vx [m/s]")
#     plt.legend()
#     plt.grid(True)

#     # Plot the X positions over time (2nd plot)
#     plt.subplot(5, 1, 2)
#     plt.plot(real_df["t"], real_df["x"], label="Real x")
#     plt.plot(sim_df["t"], sim_df["x"], label="Sim x", linestyle="--")
#     plt.ylabel("x [m]")
#     plt.legend()
#     plt.grid(True)

#     # Plot the Y positions over time (3rd plot)
#     plt.subplot(5, 1, 3)
#     plt.plot(real_df["t"], real_df["y"], label="Real y")
#     plt.plot(sim_df["t"], sim_df["y"], label="Sim y", linestyle="--")
#     plt.ylabel("y [m]")
#     plt.legend()
#     plt.grid(True)

#     # Plot the Yaw (orientation) over time (4th plot)
#     plt.subplot(5, 1, 4)
#     plt.plot(real_df["t"], real_df["yaw"], label="Real yaw")
#     plt.plot(sim_df["t"], sim_df["yaw"], label="Sim yaw", linestyle="--")
#     plt.ylabel("yaw [rad]")
#     plt.xlabel("time [s]")
#     plt.legend()
#     plt.grid(True)

#     # **Trajectory overlap plot** (5th plot)
#     plt.subplot(5, 1, 5)
#     plt.plot(real_df["x"], real_df["y"], label="Real Path")
#     plt.plot(sim_df["x"], sim_df["y"], label="Sim Path", linestyle="--")

#     # Plot settings for the trajectory plot
#     plt.title("Trajectory Overlap")
#     plt.xlabel("X [m]")
#     plt.ylabel("Y [m]")
#     plt.legend()
#     plt.grid(True)

#     # Adjust layout to make sure all plots fit properly
#     plt.tight_layout()
#     plt.show()


# ---------------------------------------------------------------------------
# Path resolution (auto-discovery)
# ---------------------------------------------------------------------------

LOG_DIR = Path.home() / "schaeffler" / "logs"
DEFAULT_REAL = LOG_DIR / "omron_run.csv"

def resolve_csv_paths(args):
    """Resolve real & sim CSV paths.

    - If --real/--sim provided: use them (and check existence).
    - Otherwise:
        real = ~/schaeffler/logs/omron_run.csv
        sim  = latest ~/schaeffler/logs/nav2_run_*.csv
    """
    log_dir = LOG_DIR

    # Real path
    real_path = Path(args.real) if args.real is not None else DEFAULT_REAL
    if not real_path.exists():
        raise SystemExit(f"[compute_kpi] Real CSV not found: {real_path}")

    # Sim path
    if args.sim is not None:
        sim_path = Path(args.sim)
        if not sim_path.exists():
            raise SystemExit(f"[compute_kpi] Sim CSV not found: {sim_path}")
    else:
        candidates = sorted(
            log_dir.glob("nav2_run_*.csv"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if not candidates:
            raise SystemExit(f"[compute_kpi] No nav2_run_*.csv found in {log_dir}")
        sim_path = candidates[0]

    print(f"[compute_kpi] Using real CSV: {real_path}")
    print(f"[compute_kpi] Using sim  CSV: {sim_path}")
    return real_path, sim_path


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--real",
        default=None,
        help=(
            "Path to real CSV. "
            "If omitted, uses ~/schaeffler/logs/omron_run.csv"
        ),
    )
    ap.add_argument(
        "--sim",
        default=None,
        help=(
            "Path to sim CSV. "
            "If omitted, uses latest nav2_run_*.csv in ~/schaeffler/logs"
        ),
    )
    ap.add_argument(
        "--pos-th",
        type=float,
        default=0.1,
        help="Position threshold [m]",
    )
    ap.add_argument(
        "--psi-th",
        type=float,
        default=0.0349,
        help="Yaw threshold [rad]",
    )
    args = ap.parse_args()

    # --- auto-resolve paths here ---
    real_path, sim_path = resolve_csv_paths(args)

    real_df = pd.read_csv(real_path)
    sim_df  = pd.read_csv(sim_path)

    for needed in ["t", "x", "y", "yaw"]:
        if needed not in real_df.columns or needed not in sim_df.columns:
            raise SystemExit(f"Missing '{needed}' column in one of the CSVs.")

    # Normalize both to their start pose & heading (common local frame)
    real_df = normalize_to_start_frame(real_df)
    sim_df  = normalize_to_start_frame(sim_df)

    rmse_pos, rmse_psi, J_tilde, hz, n = compute_kpis(
        real_df, sim_df, args.pos_th, args.psi_th
    )
    rmse_psi_deg = rmse_psi * (180.0 / math.pi)

    print(f"Samples: {n} @ ~{hz:.1f} Hz")
    print(f"RMSE_pos  [m]   = {rmse_pos:.5f}")
    print(f"RMSE_psi  [rad] = {rmse_psi:.5f}  [deg] = {rmse_psi_deg:.5f}")
    print(f"J_tilde          = {J_tilde:.5f} (<= 1.0 passes all thresholds on average)")

    # Plot normalized trajectories (so you can visually confirm alignment)
    # plot_trajectories(real_df, sim_df)


if __name__ == "__main__":
    main()
