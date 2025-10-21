from bayes_opt import BayesianOptimization
import numpy as np

rng = np.random.default_rng(42)

def compute_kpi_dummy(max_vel_x, acc_lim_x):
    # Simulate KPI (random J between 0.5 and 10)
    return float(rng.uniform(0.5, 10.0))

def evaluate(max_vel_x, acc_lim_x):
    J = compute_kpi_dummy(max_vel_x, acc_lim_x)
    print(f"trial -> max_vel_x={max_vel_x:.4f}, acc_lim_x={acc_lim_x:.4f} | J={J:.4f}")
    return -J  # BayesianOptimization maximizes, so we negate J

pbounds = {
    "max_vel_x": (0.05, 1.20),
    "acc_lim_x": (0.05, 0.30),
}

# Create optimizer
optimizer = BayesianOptimization(
    f=evaluate,
    pbounds=pbounds,
    random_state=123,
    allow_duplicate_points=True,
)

# Initialize with some random points
optimizer.maximize(init_points=3, n_iter=0)

# Manual loop to stop early when J < 1
max_total_iters = 30
for i in range(max_total_iters):
    optimizer.maximize(init_points=0, n_iter=1)
    best_target = optimizer.max["target"]
    best_J = -best_target
    print(f"[iter {i+1}] best J = {best_J:.4f} at {optimizer.max['params']}")

    if best_J < 1.0:
        print("✅ Early stop: reached J < 1")
        break

print("\n=== BO summary ===")
print("best params:", optimizer.max["params"])
print("best J     :", -optimizer.max["target"])
