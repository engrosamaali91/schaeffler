from bayes_opt import BayesianOptimization

def evaluate(max_vel_x, acc_lim_x, sim_time, xy_goal_tolerance):
    # 1. Write params into YAML (nav2_trial.yaml)
    # 2. Launch Nav2, run scenario, compute KPI J
    J = run_iteration_and_compute_J(...)
    return -J  # BO maximizes

pbounds = {
    'max_vel_x': (0.05, 1.20),
    'acc_lim_x': (0.05, 0.30),
    'sim_time': (0.8, 2.5),
    'xy_goal_tolerance': (0.02, 0.40)
}

optimizer = BayesianOptimization(
    f=evaluate,
    pbounds=pbounds,
    random_state=42,
)

# Run several iterations
optimizer.maximize(init_points=3, n_iter=10)

# You can access all previous results:
print(optimizer.res)        # list of all trials
print(optimizer.max)        # best parameters + score
# optimizer.save("bo_state.json")  # optional, if you want to resume later
