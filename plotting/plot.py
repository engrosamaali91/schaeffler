import pandas as pd
import matplotlib.pyplot as plt

# Load simulation CSV
df = pd.read_csv("real_odom_for_kpi.csv")

# --- Fix time resets by cumulative sum ---
df["t_fixed"] = df["t"].cumsum()

# -----------------------------
# Plotting
# -----------------------------
plt.figure(figsize=(12, 10))

# Subplot 1: X over time
plt.subplot(4, 1, 1)
plt.plot(df['t_fixed'], df['x'], label='X position', color='blue', marker='o', markersize=2)
plt.ylabel("X [m]")
plt.title("Robot X and Y Positions Over Time")
plt.grid(True)
plt.legend()

# Subplot 2: Y over time
plt.subplot(4, 1, 2)
plt.plot(df['t_fixed'], df['y'], label='Y position', color='orange', marker='o', markersize=2)
plt.ylabel("Y [m]")
plt.grid(True)
plt.legend()

# Subplot 3: Orientation
plt.subplot(4, 1, 3)
plt.plot(df['t_fixed'], df['yaw'], label='Theta', color='purple', marker='o', markersize=2)
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.title("Robot Orientation Over Time")
plt.grid(True)
plt.legend()

# Subplot 4: Trajectory
plt.subplot(4, 1, 4)
plt.plot(df['x'], df['y'], label='Trajectory', color='green', marker='o', markersize=2)
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Robot Trajectory (Y vs X)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
