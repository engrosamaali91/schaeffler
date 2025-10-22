import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file recorded with the odom logger
df = pd.read_csv("processed_4.csv")

# Create plots
plt.figure(figsize=(12, 8))

# x over time
plt.subplot(3, 2, 1)
plt.plot(df["t"], df["x"])
plt.xlabel("Time [s]")
plt.ylabel("X [m]")
plt.title("X over Time")

# y over time
plt.subplot(3, 2, 2)
plt.plot(df["t"], df["y"])
plt.xlabel("Time [s]")
plt.ylabel("Y [m]")
plt.title("Y over Time")

# yaw over time
plt.subplot(3, 2, 3)
plt.plot(df["t"], df["yaw"])
plt.xlabel("Time [s]")
plt.ylabel("Yaw [rad]")
plt.title("Yaw over Time")

# xy trajectory
plt.subplot(3, 2, 4)
plt.plot(df["x"], df["y"])
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("XY Trajectory")
plt.axis("equal")

# velocity over time
plt.subplot(3, 2, 5)
plt.plot(df["t"], df["vx"])
plt.xlabel("Time [s]")
plt.ylabel("Velocity X [m/s]")
plt.title("Velocity over Time")

plt.tight_layout()
plt.show()
