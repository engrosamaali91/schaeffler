## 📘 Robot Pose Data (LD-250 Real Robot)

This table describes the meaning and units of the fields published by the **Omron LD-250** on the MQTT topic  
`itk/dt/robot/pose`.  
These values represent the robot’s real-world position and orientation, expressed in milli-scaled SI units.

| Field | Description | Default Unit (MQTT message) | SI Equivalent | Conversion Formula |
|--------|--------------|-----------------------------|----------------|--------------------|
| `x` | Robot position along X-axis | **millimeters (mm)** | meters (m) | `x_m = x / 1000` |
| `y` | Robot position along Y-axis | **millimeters (mm)** | meters (m) | `y_m = y / 1000` |
| `th` | Robot orientation (theta / yaw) | **degrees** | radians (rad) | `th_rad = th / 1000` |
| `upd` | Timestamp of pose update | **unix timestamp** | seconds (s) | `t_s = upd / 1000` |



`macro_setSpeeed.py` calls the macro

`macro_logger` logs the data `t x y yaw v_x` after conversions

`plot_robot_csv.py` do the transformation, product a plot as show in image ![plot](all_plots_combined.png) and final csv file for KPI computation 