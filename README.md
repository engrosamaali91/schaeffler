# Steps for Converting Data, Preprocessing, and Running `compute_kpis.py`

## Step 1: **Converting Simulated Data (rosbag) to CSV**

1. **Command used**:
   - Convert the ROS bag file to CSV using extract_rosbagdata.py file and then preprocess the data using preprocess.py. 
   
2. **Preprocessing the data**:
   - **Trimming**: Using velocity (`vx > 0.02 m/s`) and goal proximity (goal reached when `x, y` within 0.05 m of target).
   - **Zeroing**: Subtract the first timestamp (`t0`) to start at **`t=0`** for both sim and real data.
   - ```python preprocess.py```

3. Plotting 
   - Plot using python plot.py script to visualize the plot
---

## Step 2: **Recording Real Robot Data Using MQTT**

1. **Setup**:
   - MQTT client connects to the robot at `HOST = 192.168.18.3` on port `8883` using TLS for security.

2. **Data Logging**:
   - Subscribe to pose and status topics: `itk/dt/robot/pose` and `itk/dt/robot/status`.
   - Log data only when `status="Driving"` (motion is active).
   - ```python mqtt.py```

3. **Data Format**:
   - Log format: `t, x, y, yaw` (where `t` is in seconds, `x, y` in meters, and `yaw` in radians).

4. **MQTT Data Handling**:
   - When a status update is received, check if the robot is moving (`status=="Driving"`).
   - On receiving pose data (`/pose` topic), write the `t,x,y,yaw` values to a CSV file.

---

## Step 3: **Running `compute_kpis.py`**

1. **Input**:  
   - **Real and Sim CSVs**: Both datasets are in the format of `t,x,y,yaw`.

2. **Preprocessing before KPIs**:
   - Normalize both datasets to the same start pose and initial yaw. This aligns both real and simulated data in the same coordinate frame (zeroed at the start position, initial yaw set to 0).

3. **Key Functionality**:
   - **Resampling**: The script resamples both the real and sim data to **10 Hz** based on the overlapping time range.
   - **KPIs Computation**: Computes **RMSE_pos**, **RMSE_psi**, and **J_tilde**:
     - **RMSE_pos**: Position error between real and sim.
     - **RMSE_psi**: Yaw error.
     - **J_tilde**: Normalized combined error score.

4. **Running the script**:
   - Example command:
     ```bash
     python compute_kpis.py --real real_log_for_kpi.csv --sim sim_log_for_kpi.csv
     ```

---

## Step 4: **Running the Entire Process**

1. **Data Logging (Real Robot)**:
   - Run the MQTT script that logs real robot data and outputs to `real_log_for_kpi.csv`.

2. **Simulated Data Conversion**:
   - Convert simulated rosbag data to `sim_log_for_kpi.csv`.

3. **Run `compute_kpis.py`**:
   - Use the following command to compute the KPIs:
     ```bash
     python compute_kpis.py --real real_log_for_kpi.csv --sim sim_log_for_kpi.csv
     ```

4. **Output**:
   - Prints RMSE values for position (`RMSE_pos`), yaw (`RMSE_psi`), and combined error (`J_tilde`).


# Finding: Adjusting Tolerances Based on Omron LD250 Datasheet

## Problem:
When comparing the real robot data with the simulated data, **J_tilde** was high due to the large position error (1.08 m) in the real robot's movement. This was after setting the default thresholds for position and yaw based on initial assumptions.

## Approach:
1. **Original Thresholds**:
   - **Position Tolerance (`T_pos`)**: 0.03 m (3 cm)
   - **Yaw Tolerance (`T_psi`)**: 0.02 rad (~1.15°)

2. **Omron LD250 Datasheet Thresholds**:
   - **Position Tolerance (`T_pos`)**: 0.1 m (100 mm)
   - **Yaw Tolerance (`T_psi`)**: 0.0349 rad (2°)

## Results:

### **Before Adjusting to LD250 Thresholds**:
```bash
RMSE_pos  [m]  = 1.08025
RMSE_psi  [rad] = 0.02637  [deg] = 1.51065
J_tilde         = 18.66327
```

### **After Adjusting to LD250 Thresholds**:
```
RMSE_pos  [m]  = 1.08025
RMSE_psi  [rad] = 0.02637  [deg] = 1.51065
J_tilde         = 5.77897
```

J_tilde = 5.77897 shows a significant reduction in J_tilde after increasing the position threshold to 0.1 m and yaw to 2°, as per the Omron LD250 datasheet.



### KPI Evaluation Results of pilot iteration 

| Metric | Description | Value | Interpretation |
|---------|--------------|--------|----------------|
| **Overlap** | Common time window between real and simulated trajectories | 0.00 – 9.90 s | 10 s of overlapping data used for comparison |
| **Sim Δx, Δy [m]** | Net displacement of simulated robot in X/Y | 2.937, −0.018 | Sim robot moved ~3 m forward |
| **Real Δx, Δy [m]** | Net displacement of real robot in X/Y | 0.921, −0.012 | Real robot moved ~1 m forward |
| **Samples** | Number of synchronized samples at ~10 Hz | 100 | Sufficient data for KPI computation |
| **RMSE_pos [m]** | Root Mean Square Error of position | **2.018** | Large spatial deviation between sim and real |
| **RMSE_psi [rad] / [deg]** | Root Mean Square Error of yaw | **0.0175 rad (≈ 1.0°)** | Good heading agreement |
| **J_tilde** | Combined normalized KPI (≤ 1.0 = pass) | **10.34** | Fails threshold — significant sim-to-real gap in position |

**Summary:**  
Simulation shows similar heading but moves ~3× farther than the real robot, leading to a high position RMSE and large `J_tilde`.  
Further tuning of simulated robot dynamics or control parameters is required to reduce the sim-to-real gap.
