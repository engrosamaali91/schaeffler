#!/usr/bin/env python3
"""
compute_kpis.py

Usage:
  python compute_kpis.py --real real_log.csv --sim sim_log.csv \
      [--pos-th 0.03] [--psi-th 0.02]

CSV format expected (headers required):
  t,x,y,yaw
  - t in seconds (monotonic, float)
  - x,y in meters (map/world frame)
  - yaw in radians (map/world frame)

Outputs:
  Prints RMSE_pos [m], RMSE_psi [rad], J_tilde to stdout.
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt




def angle_wrap_diff(a, b):
    d = a - b
    return np.arctan2(np.sin(d), np.cos(d))

def rmse(arr):
    return float(np.sqrt(np.mean(np.square(arr))))

def resample_to(common_t, df):
    out = {}
    for col in df.columns:
        if col == "t":
            continue
        out[col] = np.interp(common_t, df["t"].to_numpy(), df[col].to_numpy())
    out_df = pd.DataFrame({"t": common_t, **out})
    return out_df

# def compute_velocity_from_position(t, x):
#     # Deprecated: velocity not used in current KPI computation.
#     t = np.asarray(t, dtype=float)
#     x = np.asarray(x, dtype=float)
#     dt = np.gradient(t)
#     vx = np.gradient(x) / dt
#     return vx

def compute_kpis(real_df, sim_df, T_pos, T_psi):
    # Overlapping time window
    t_min = max(real_df["t"].min(), sim_df["t"].min())
    t_max = min(real_df["t"].max(), sim_df["t"].max())
    if t_max <= t_min:
        raise ValueError("Time ranges do not overlap between real and sim.")
    # Common time grid (5..50 Hz)
    # hz_real = (len(real_df) - 1) / (real_df["t"].max() - real_df["t"].min())
    # hz_sim = (len(sim_df) - 1) / (sim_df["t"].max() - sim_df["t"].min())
    # hz = max(5.0, min(50.0, max(hz_real, hz_sim)))
    hz = 10.0  # resample to real robot frequency
    common_t = np.arange(t_min, t_max, 1.0/hz)

    real_i = resample_to(common_t, real_df)
    sim_i  = resample_to(common_t, sim_df)

    # Position RMSE
    dx = sim_i["x"].to_numpy() - real_i["x"].to_numpy()
    dy = sim_i["y"].to_numpy() - real_i["y"].to_numpy()
    rmse_pos = rmse(np.sqrt(dx*dx + dy*dy))

    # Yaw RMSE
    dyaw = angle_wrap_diff(sim_i["yaw"].to_numpy(), real_i["yaw"].to_numpy())
    rmse_psi = rmse(dyaw)

    J_tilde = float(np.mean([rmse_pos / T_pos, rmse_psi / T_psi]))
    return rmse_pos, rmse_psi, J_tilde, hz, len(common_t)




def plot_trajectories(real_df, sim_df):
    plt.figure(figsize=(12, 8))

    # X
    plt.subplot(3, 1, 1)
    plt.plot(real_df["t"], real_df["x"], label="Real x")
    plt.plot(sim_df["t"], sim_df["x"], label="Sim x", linestyle="--")
    plt.ylabel("x [m]")
    plt.legend()
    plt.grid(True)

    # Y
    plt.subplot(3, 1, 2)
    plt.plot(real_df["t"], real_df["y"], label="Real y")
    plt.plot(sim_df["t"], sim_df["y"], label="Sim y", linestyle="--")
    plt.ylabel("y [m]")
    plt.legend()
    plt.grid(True)

    # Yaw
    plt.subplot(3, 1, 3)
    plt.plot(real_df["t"], real_df["yaw"], label="Real yaw")
    plt.plot(sim_df["t"], sim_df["yaw"], label="Sim yaw", linestyle="--")
    plt.ylabel("yaw [rad]")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", required=True, help="Path to real CSV")
    ap.add_argument("--sim", required=True, help="Path to sim CSV")
    ap.add_argument("--pos-th", type=float, default=0.03, help="Position threshold [m]")
    ap.add_argument("--psi-th", type=float, default=0.02, help="Yaw threshold [rad]")
    args = ap.parse_args()

    real_df = pd.read_csv(args.real)
    sim_df  = pd.read_csv(args.sim)

    for needed in ["t","x","y","yaw"]:
        if needed not in real_df.columns or needed not in sim_df.columns:
            raise SystemExit(f"Missing '{needed}' column in one of the CSVs.")

    rmse_pos, rmse_psi, J_tilde, hz, n = compute_kpis(real_df, sim_df, args.pos_th, args.psi_th)
    print(f"Samples: {n} @ ~{hz:.1f} Hz")
    print(f"RMSE_pos  [m]  = {rmse_pos:.5f}")
    print(f"RMSE_psi  [rad]= {rmse_psi:.5f}")
    print(f"J_tilde         = {J_tilde:.5f} (<= 1.0 passes all thresholds on average)")
    plot_trajectories(real_df, sim_df)

if __name__ == "__main__":
    main()
