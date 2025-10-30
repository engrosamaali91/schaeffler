## 🚀 Nav2 Parameter Optimization Progress

### 🧩 Goal
Automate Nav2 parameter tuning to bridge the **Sim2Real gap** by:
- Reading parameters from the base `nav2_params.yaml`
- Overwriting selected parameters (manually or automatically)
- Dumping updated parameters into a new file (`nav2_trialXX.yaml`)
- Launching Nav2 with the new file and measuring KPI (**J** value)

---

### ⚙️ 1. Manual Parameter Override
**Script:** `nav2_yaml_override.py`  
Loads an existing parameter file, overwrites selected keys, and saves a new one.

```bash
python3 nav2_yaml_override.py \
  --in src/GetSetParams/nav2_trial02.yaml \
  --out src/GetSetParams/nav2_trial03.yaml \
  --set \
    controller_server.FollowPath.max_vel_x=0.28 \
    controller_server.FollowPath.acc_lim_x=2.3 \
    controller_server.FollowPath.sim_time=1.6
```

### Automated Parameter Proposal (Optimizer)
```bash
python3 nav2_optimize_and_override.py \
  --in nav2_params.yaml \
  --out nav2_trial.yaml \
  --keys \
    controller_server.FollowPath.max_vel_x \
    controller_server.FollowPath.acc_lim_x \
    controller_server.FollowPath.max_vel_theta \
    controller_server.FollowPath.acc_lim_theta \
    controller_server.FollowPath.sim_time \
    controller_server.FollowPath.xy_goal_tolerance \
  --j 1.34 \
  --state optimizer_state.json \
  --lower 0.0 --upper 2.0 \
  --seed 42
```



### Parameter Initialization
All initial parameters in `nav2_params.yaml` are derived from the real robot’s configuration:

| Robot UI Param | Converted Nav2 Param | Units | Source |
|----------------|----------------------|--------|---------|
| TransVelMax = 1200 mm/s | max_vel_x = 1.20 | m/s | MobilePlanner |
| TransAccel = 300 mm/s² | acc_lim_x = 0.30 | m/s² | MobilePlanner |
| RotVelMax = 60 deg/s | max_vel_theta = 1.05 | rad/s | MobilePlanner |
| RotAccel = 90 deg/s² | acc_lim_theta = 1.57 | rad/s² | MobilePlanner |

### Bounds
`bounds.yaml` defines allowed search space for the optimizer based on the same physical limits of the real robot.

### Optimization Loop
1. Start with `nav2_params.yaml`
2. Run Nav2 → compute J
3. Optimizer proposes new params within `bounds.yaml`
4. Write `nav2_trialXX.yaml`
5. Relaunch → recompute J → iterate



| Robot UI Parameter (from MobilePlanner) | Meaning / Units                    | Converted Nav2 Parameter(s)                                                         | Converted Value (from Robot)              | Default Value (Nav2 file) |
| --------------------------------------- | ---------------------------------- | ----------------------------------------------------------------------------------- | ----------------------------------------- | ------------------------- |
| **TransVelMax = 1200 mm/s**             | Max forward translational velocity | `controller_server.FollowPath.max_vel_x`<br>`velocity_smoother.max_velocity[0]`     | **1.20 m/s**                              | **0.26 m/s**              |
| **TransNegVelMax = -200 mm/s**          | Max reverse translational velocity | `controller_server.FollowPath.min_vel_x`<br>`velocity_smoother.min_velocity[0]`     | **-0.20 m/s**                             | **-0.26 m/s**             |
| **TransAccel = 300 mm/s²**              | Max linear acceleration            | `controller_server.FollowPath.acc_lim_x`<br>`velocity_smoother.max_accel[0]`        | **0.30 m/s²**                             | **2.50 m/s²**             |
| **TransDecel = 600 mm/s²**              | Max linear deceleration            | `controller_server.FollowPath.decel_lim_x`<br>`velocity_smoother.max_decel[0]`      | **-0.60 m/s²** (negative in controller)   | **-2.50 m/s²**            |
| **RotVelMax = 60°/s**                   | Max angular velocity               | `controller_server.FollowPath.max_vel_theta`<br>`velocity_smoother.max_velocity[2]` | **1.05 rad/s**                            | **1.00 rad/s**            |
| **RotAccel = 90°/s²**                   | Angular acceleration               | `controller_server.FollowPath.acc_lim_theta`<br>`velocity_smoother.max_accel[2]`    | **1.57 rad/s²**                           | **3.20 rad/s²**           |
| **RotDecel = 90°/s²**                   | Angular deceleration               | `controller_server.FollowPath.decel_lim_theta`<br>`velocity_smoother.max_decel[2]`  | **-1.57 rad/s²** (negative in controller) | **-3.20 rad/s²**          |
| *(derived)*                             | Goal tolerance radius              | `controller_server.FollowPath.xy_goal_tolerance`                                    | —                                         | **0.25 m**                |
| *(derived)*                             | Time horizon for local planner     | `controller_server.FollowPath.sim_time`                                             | —                                         | **1.7 s**                 |
| *(derived)*                             | Linear stop detection threshold    | `controller_server.FollowPath.trans_stopped_velocity`                               | ~0.05 × `max_vel_x` ≈ **0.06 m/s**        | **0.25 m/s**              |



## ✅ In Summary — Key Motion Parameters

| Term | Intuitive Meaning | Formula / Effect |
|------|--------------------|------------------|
| **max_vel_x** | “How fast can I go?” | Defines the **maximum linear speed** the robot can reach (m/s). |
| **acc_lim_x** | “How quickly can I get there?” | Sets the **rate of acceleration** (m/s²). A higher value → snappier motion; lower value → smoother, slower ramp-up. |
| **Ramp time** | “How long does it take to reach top speed?” | ```t = v_max / a_lim```  →  shorter = snappier motion, longer = smoother motion |



### Concept

During navigation, the robot begins each motion at 0 m/s and ramps its velocity until reaching max_vel_x, respecting the limit imposed by ```acc_lim_x```.
The pair (```max_vel_x, acc_lim_x```) directly shapes how aggressively or smoothly the robot accelerates and decelerates in simulation versus reality.

In the Sim2Real optimization, these are the primary tuning knobs:

```max_vel_x``` controls the steady-state speed the simulated AGV maintains.

```acc_lim_x``` controls the shape and timing of the velocity curve (the ramp).

The goal of optimization is to find parameter values that reproduce the real robot’s acceleration profile and motion timing inside simulation, minimizing the measured Sim2Real gap (J).


### ⚠️ Limitations of the Adaptive Step-Size Optimizer

This optimizer adapts **all parameter values** based on a **single global KPI (J)** — it does not know which individual key caused the improvement or degradation.

| Aspect | Description |
|--------|--------------|
| **Global feedback** | Uses one J value for all keys; if J worsens, all parameters are adjusted (even if only one caused it). |
| **No parameter interaction modeling** | Does not learn relationships or coupling between parameters (e.g., `max_vel_x` ↔ `acc_lim_x`). |
| **Slow fine-tuning** | Random perturbations and global updates can require many iterations to converge. |

Despite this, the method is **lightweight, derivative-free, and adaptive**, making it ideal for early-stage Sim2Real tuning when only a few parameters are optimized.
