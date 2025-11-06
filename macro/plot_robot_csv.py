import pandas as pd
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
# ⚠️ IMPORTANT: Change this to the exact name of your CSV file.
CSV_FILENAME = "real_log_setSpeed_1762438101.csv" # Example: "real_log_setSpeed_1234567890.csv"
# ---------------------

def analyze_and_plot_robot_data(csv_file):
    """
    Loads robot pose/velocity data from a CSV, transforms the X and Y
    columns to be relative to the start (0,0), saves this transformed
    data to a new CSV, and plots all results into a single combined image.
    """
    try:
        # 1. Load Data
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found.")
        print("Please make sure the CSV_FILENAME variable is set correctly.")
        return
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # 2. Data Cleaning & Transformation
    
    # Handle the first 'vx' row, which is blank/NaN
    df['vx'] = df['vx'].fillna(0.0)

    # Get initial values
    initial_x = df['x'].iloc[0]
    initial_y = df['y'].iloc[0]

    print(f"Successfully loaded '{csv_file}' ({len(df)} data points).")
    print(f"Original Start Position: X={initial_x:.3f}m, Y={initial_y:.3f}m")

    # --- THIS IS THE KEY CHANGE ---
    # Overwrite the 'x' and 'y' columns with relative (starting at 0) values
    df['x'] = df['x'] - initial_x
    df['y'] = df['y'] - initial_y
    # -----------------------------

    print(f"Transformed 'x' and 'y' columns to start from (0, 0).")

    # --- 3. Save Transformed Data to New CSV ---
    # Create a new filename for the transformed data
    transformed_csv_name = f"transformed_{csv_file}"
    try:
        # Save the DataFrame. It now has the transformed x,y columns
        # and no extra 'distance_x' or 'y_displacement' columns.
        df.to_csv(transformed_csv_name, index=False, float_format='%.6f')
        print(f"\nSaved transformed data to: {transformed_csv_name}")
    except Exception as e:
        print(f"Error saving transformed CSV: {e}")
        
    # --- 4. Plotting (All in one Figure) ---
    
    # Create a 3x2 grid of subplots
    fig, ax = plt.subplots(3, 2, figsize=(18, 18)) # Changed from 2x2 to 3x2
    fig.suptitle(f'Robot Performance Analysis (Relative): {csv_file}', fontsize=16, y=0.98)

    # --- Plot 1 (Position 1,1): Forward Distance Traveled vs Time (X over Time) ---
    ax[0, 0].plot(df['t'], df['x'], marker='.', linestyle='-', markersize=4, color='blue')
    ax[0, 0].set_xlabel('Time (s)')
    ax[0, 0].set_ylabel('Distance Traveled (m)')
    ax[0, 0].set_title('Forward Distance Traveled Over Time')
    ax[0, 0].grid(True)

    # --- Plot 2 (Position 1,2): Y Position (Drift) vs Time (Y over Time) ---
    ax[0, 1].plot(df['t'], df['y'], marker='.', linestyle='-', markersize=4, color='green')
    ax[0, 1].axhline(0, color='gray', linestyle='--', linewidth=1)
    ax[0, 1].set_xlabel('Time (s)')
    ax[0, 1].set_ylabel('Relative Y Position (Drift) (m)')
    ax[0, 1].set_title('Y Position (Drift) Over Time')
    ax[0, 1].grid(True)

    # --- Plot 3 (Position 2,1): Heading (Yaw) vs Time ---
    ax[1, 0].plot(df['t'], df['yaw'], marker='.', linestyle='-', markersize=4, color='orange')
    ax[1, 0].axhline(0, color='gray', linestyle='--', linewidth=1)
    ax[1, 0].set_xlabel('Time (s)')
    ax[1, 0].set_ylabel('Yaw (Heading) in Radians')
    ax[1, 0].set_title('Robot Heading (Yaw) Over Time')
    ax[1, 0].grid(True)

    # --- Plot 4 (Position 2,2): Relative Trajectory (XY Trajectory) ---
    ax[1, 1].plot(df['x'], df['y'], marker='o', linestyle='-', markersize=2, label='Robot Path')
    ax[1, 1].scatter(df['x'].iloc[0], df['y'].iloc[0], color='green', s=100, zorder=5, label=f'Start ({df["x"].iloc[0]:.1f}, {df["y"].iloc[0]:.1f})')
    ax[1, 1].scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='red', s=100, zorder=5, label=f'End ({df["x"].iloc[-1]:.3f}m)')
    ax[1, 1].axhline(0, color='gray', linestyle='--', linewidth=0.8) # Reference line
    ax[1, 1].set_xlabel('Relative X Position (m)')
    ax[1, 1].set_ylabel('Relative Y Position (Drift) (m)')
    ax[1, 1].set_title('Robot Trajectory (Relative to Start)')
    ax[1, 1].axis('equal') # Keep scale consistent
    ax[1, 1].legend()
    ax[1, 1].grid(True)

    # --- Plot 5 (Position 3,1): Linear Velocity (vx) vs Time ---
    ax[2, 0].plot(df['t'], df['vx'], marker='.', linestyle='-', markersize=4, color='purple')
    max_speed = df['vx'].max()
    ax[2, 0].axhline(max_speed, color='red', linestyle=':', linewidth=1, label=f'Max Speed: {max_speed:.3f} m/s')
    ax[2, 0].set_xlabel('Time (s)')
    ax[2, 0].set_ylabel('Forward Velocity (m/s)')
    ax[2, 0].set_title('Calculated Forward Velocity (vx) Over Time')
    ax[2, 0].legend()
    ax[2, 0].grid(True)

    # --- Hide the empty 6th plot (ax[2, 1]) ---
    ax[2, 1].axis('off')

    # Adjust layout to prevent overlap and save the combined figure
    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust for suptitle
    
    combined_plot_filename = 'all_plots_combined.png'
    plt.savefig(combined_plot_filename)
    print(f"Saved all plots to: {combined_plot_filename}")

    # --- 5. Print Summary ---
    total_travel_time = df['t'].iloc[-1]
    total_travel_distance = df['x'].iloc[-1] # Now use 'x'
    max_drift = df['y'].abs().max()         # Now use 'y'
    avg_speed_overall = total_travel_distance / total_travel_time
    
    print("\n--- Summary ---")
    print(f"Total time taken: {total_travel_time:.3f} seconds")
    print(f"Total distance traveled (X-axis): {total_travel_distance:.3f} meters")
    print(f"Max recorded speed (vx): {max_speed:.3f} m/s")
    print(f"Overall average speed: {avg_speed_overall:.3f} m/s")
    print(f"Max lateral drift (Y-axis): {max_drift * 1000:.1f} mm")


# --- EXECUTION ---
if __name__ == "__main__":
    # ⚠️ REMINDER: Update CSV_FILENAME to your log file
    analyze_and_plot_robot_data(CSV_FILENAME)