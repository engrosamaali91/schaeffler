import matplotlib.pyplot as plt

# Example J_tilde values and corresponding tuning parameters for each iteration
iterations = [1, 2, 3, 4, 5]
j_tilde_values = [13.83, 7.07, 5.62, 4.22, 3.18]
max_vel_x_values = [1.2, 0.5, 0.6, 0.5, 0.4]  # Example values for max_vel_x
acc_lim_x_values = [2.5, 2.5, 1.2, 0.9, 0.7]  # Example values for acc_lim_x

# Bar plot for J_tilde over iterations
plt.figure(figsize=(10, 6))
bars = plt.bar(iterations, j_tilde_values, color='skyblue')

# Labels and title
plt.xlabel("Iteration")
plt.ylabel("J_tilde")
plt.title("J_tilde over Time/Iterations with Parameter Changes")
plt.xticks(iterations)  # To label each bar with the iteration number
plt.grid(True, axis='y', linestyle='--', alpha=0.7)

# Annotate each bar with multiple parameter values inside
for i, bar in enumerate(bars):
    # Get the height of the bar (J_tilde value)
    height = bar.get_height()
    
    # Format the text with multiple parameters inside the bar
    text = f"max_vel_x={max_vel_x_values[i]:.2f}\nacc_lim_x={acc_lim_x_values[i]:.2f}"

    # Place the text inside the bar
    plt.text(bar.get_x() + bar.get_width() / 2, height / 2,  # X position (center of the bar), Y position (middle of the bar)
             text,                                           # Text content with parameter values
             ha='center', va='center', fontsize=10, color='white')

# Show the plot
plt.tight_layout()
plt.show()
