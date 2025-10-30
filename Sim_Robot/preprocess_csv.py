#!/usr/bin/env python3
import pandas as pd
import argparse
import sys

def preprocess(input_csv, output_csv, goal_x, goal_y, vx_thresh, goal_tol, consec):
    """
    Summary:
    
    1. **Reading and Preparing Data**:
       - Load the input CSV into a pandas DataFrame.
       - Ensure required columns: `t`, `x`, `y`, `yaw`, `vx`.
       - Convert all values to numeric, coercing errors to NaN, then drop NaN rows.
    
    2. **Motion Start Detection**:
       - Create a "moving" column based on the velocity (`vx > vx_thresh`).
       - Use a rolling sum to detect if there are `consec` consecutive "moving" samples.
       - If motion is detected, identify the starting index `i0`, otherwise, zero time and save.
    
    3. **Motion End Detection**:
       - Detect when the robot reaches the goal (`x`, `y` close to `goal_x`, `goal_y`).
       - Check for "still" condition (`vx < vx_thresh`) for `consec` consecutive samples.
       - If stopped and near goal, detect end index `i1`.
       - Trim the DataFrame to include only motion data.
    
    4. **Zeroing Time**:
       - Subtract the first recorded timestamp `t0` from the entire `t` column so that the time starts from `t=0`.
    
    5. **Write Cleaned Data**:
       - Save the processed DataFrame with only `t`, `x`, `y`, `yaw` columns to the output CSV.
    """
    
    # Read the input CSV file into a DataFrame
    df = pd.read_csv(input_csv)

    # Ensure required columns exist
    required = {"t","x","y","yaw","vx"}
    missing = required - set(df.columns)
    if missing:
        print(f"ERROR: Missing columns in {input_csv}: {sorted(missing)}", file=sys.stderr)
        sys.exit(1)

    # Convert each column to numeric type 
    for c in ["t","x","y","yaw","vx"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    # remove any rows with NaN 
    df = df.dropna(subset=["t","x","y","yaw","vx"]).reset_index(drop=True)

    # --- Detect motion start ---
    moving = (df["vx"].abs() > vx_thresh).astype(int)
    run = moving.rolling(consec, min_periods=1).sum()
    if not (run == consec).any():
        df_out = df.copy()
        df_out["t"] -= df_out["t"].iloc[0]
        df_out.loc[:, ["t","x","y","yaw"]].to_csv(output_csv, index=False)
        print(f"⚠️ No motion detected. Wrote zeroed log {output_csv} with {len(df_out)} rows.")
        return
    i0 = int(run[run == consec].index[0]) #motion starts

    # --- Detect motion end ---
    near_goal = (df["x"].sub(goal_x).abs() < goal_tol) & (df["y"].sub(goal_y).abs() < goal_tol)
    still = (df["vx"].abs() < vx_thresh).astype(int)
    stop = (still.rolling(consec, min_periods=1).sum() == consec) & near_goal
    i1 = int(stop[stop].index[-1]) if stop.any() else int(df.index[-1]) #motion stops

    if i1 < i0:
        i1 = int(df.index[-1])

    seg = df.loc[i0:i1].copy()

    # --- Zero time ---
    t0 = seg["t"].iloc[0]
    seg["t"] = seg["t"] - t0

    # --- Write KPI-compatible file ---
    seg.loc[:, ["t","x","y","yaw", "vx"]].to_csv(output_csv, index=False)
    print(f"✅ Saved KPI file {output_csv}: {len(seg)} rows, duration {seg['t'].iloc[-1]:.3f}s")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="input_csv", default="odom_data_new.csv")
    ap.add_argument("--out", dest="output_csv", default="odom_for_kpi.csv")
    ap.add_argument("--goal-x", type=float, default=3.0) # the goal coordinates to compare against.
    ap.add_argument("--goal-y", type=float, default=0.0) # the goal coordinates to compare against.
    ap.add_argument("--vx-th", type=float, default=0.01) # the threshold for velocity to detect if the robot is moving.
    ap.add_argument("--goal-tol", type=float, default=0.05) #tolerance for proximity to the goal.
    ap.add_argument("--consec", type=int, default=3) #number of consecutive samples needed to detect motion.
    args = ap.parse_args()

    preprocess(args.input_csv, args.output_csv,
               args.goal_x, args.goal_y,
               args.vx_th, args.goal_tol, args.consec)

if __name__ == "__main__":
    main()
