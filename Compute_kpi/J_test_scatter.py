import matplotlib.pyplot as plt

# Data
iterations = [1, 2, 3, 4, 5, 6]
j_tilde_values = [45.53, 13.83, 7.07, 6.63, 2.5, 1.1]

# ROS
ros_max_vel_x = [0.26, 0.26, 0.26, 0.2, 0.4, 0.2]
ros_max_acc_x = [2.5, 2.5, 2.5, 0.2, 0.4, 0.1]

# Isaac Sim
isaac_max_vel_x = [1.2, 1.2, 0.5, 0.0, 0.0, 0.0]
isaac_max_acc_x = [1.0472, 1.0472, 1.0472, 0.0, 0.0, 0.0]



# pos_th = [0.03, 0.1, 0.1,0.1,0.1,0.1 ]
# pos_psi = [0.02, 0.0349, 0.0349,0.0349,0.0349,0.0349 ]


# Plot
fig, ax1 = plt.subplots(figsize=(11, 6))

# J_tilde on left Y-axis
ax1.plot(iterations, j_tilde_values, marker='o', color='blue', linewidth=2, label='J_tilde')
ax1.set_xlabel("Iteration")
ax1.set_ylabel("J_tilde", color='blue')
ax1.tick_params(axis='y', labelcolor='blue')

# Right Y-axis (vel & accel)
ax2 = ax1.twinx()

# Velocity:
ax2.plot(iterations, ros_max_vel_x, marker='s', linestyle='--', color='green', label='ROS max_vel_x')
ax2.plot(iterations, isaac_max_vel_x, marker='D', linestyle='-.', color='orange', label='Isaac max_vel_x')

# Acceleration:
ax2.plot(iterations, ros_max_acc_x, marker='^', linestyle=':', color='red', label='ROS max_acc_x')
ax2.plot(iterations, isaac_max_acc_x, marker='x', linestyle='-', color='purple', label='Isaac max_acc_x')

ax2.set_ylabel("Velocity (m/s) & Acceleration (m/s²)", color='black')
ax2.tick_params(axis='y', labelcolor='black')

# Combined Legend
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper right')

# Title + grid
plt.title("J_tilde vs ROS & Isaac Sim Parameters Over Iterations")
ax1.grid(True)
plt.tight_layout()
plt.show()
