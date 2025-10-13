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
`bounds.yaml` defines allowed search space for the optimizer based on the same physical limits.

### Optimization Loop
1. Start with `nav2_params.yaml`
2. Run Nav2 → compute J
3. Optimizer proposes new params within `bounds.yaml`
4. Write `nav2_trialXX.yaml`
5. Relaunch → recompute J → iterate
