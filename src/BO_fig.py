# ---------- Visualize a bayes_opt run (copy this into a Jupyter cell or a .py script) ----------
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from bayes_opt import BayesianOptimization
import pandas as pd

# 1) Your objective
def black_box_function(x, y):
    return -x ** 2 - (y - 1) ** 2 + 1

# 2) Your bounds
pbounds = {'x': (2, 4), 'y': (-3, 3)}

# 3) Run Bayesian Optimization as you did
optimizer = BayesianOptimization(
    f=black_box_function,
    pbounds=pbounds,
    random_state=1,
)

init_points = 2
n_iter = 10

optimizer.maximize(
    init_points=init_points,
    n_iter=n_iter,
)

# 4) Extract the full trace of evaluations
xs, ys, ts = [], [], []
for r in optimizer.res:
    xs.append(r["params"]["x"])
    ys.append(r["params"]["y"])
    ts.append(r["target"])

xs = np.array(xs)
ys = np.array(ys)
ts = np.array(ts)
iters = np.arange(1, len(ts)+1)
best_so_far = np.maximum.accumulate(ts)

print("\nBest found:", optimizer.max)


# 5) Create a grid to visualize the objective over your bounds
x_min, x_max = pbounds['x']
y_min, y_max = pbounds['y']
gx = np.linspace(x_min, x_max, 120)
gy = np.linspace(y_min, y_max, 150)
GX, GY = np.meshgrid(gx, gy)
def fgrid(X, Y): return -X**2 - (Y-1)**2 + 1
Z = fgrid(GX, GY)

# Constrained maximum in these bounds (analytically): (x=2, y=1)
x_star_c, y_star_c = 2.0, 1.0
z_star_c = black_box_function(x_star_c, y_star_c)

# 6) 3D surface with sampled points
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.plot_surface(GX, GY, Z, alpha=0.7)
# Initial random points (squares) vs BO-chosen points (circles)
ax1.scatter(xs[:init_points], ys[:init_points], ts[:init_points], s=70, marker='s')
ax1.scatter(xs[init_points:], ys[init_points:], ts[init_points:], s=70, marker='o')
# Mark constrained optimum
ax1.scatter([x_star_c], [y_star_c], [z_star_c], s=90, marker='X')
ax1.set_xlabel("x"); ax1.set_ylabel("y"); ax1.set_zlabel("f(x,y)")
ax1.set_title("Objective Surface + Samples")
plt.show()

# 7) 2D contour with sampling order
fig2 = plt.figure()
cs = plt.contour(GX, GY, Z, levels=25)
plt.clabel(cs, inline=True, fontsize=8)
# Draw the bounds (for clarity)
plt.axvline(x_min); plt.axvline(x_max); plt.axhline(y_min); plt.axhline(y_max)
# Plot samples with different markers
plt.scatter(xs[:init_points], ys[:init_points], s=60, marker='s', label='init (random)')
plt.scatter(xs[init_points:], ys[init_points:], s=60, marker='o', label='BO (guided)')
# Label order
for i, (xi, yi) in enumerate(zip(xs, ys)):
    plt.text(xi, yi, str(i), fontsize=9, ha='left', va='bottom')
# Mark constrained optimum
plt.scatter([x_star_c], [y_star_c], s=80, marker='X', label='constrained optimum')
plt.xlabel("x"); plt.ylabel("y"); plt.title("Contours + Sampling Order")
plt.legend()
plt.show()

# 8) Learning curve: observed target and best-so-far
fig3 = plt.figure()
plt.plot(iters, ts, marker='o', label='Observed')
plt.plot(iters, best_so_far, marker='o', linestyle='--', label='Best so far')
plt.xlabel("Iteration"); plt.ylabel("f(x,y)")
plt.title("Bayesian Optimization Progress")
plt.legend()
plt.show()

# 9) Table/CSV of results

df = pd.DataFrame({"iteration": iters-1, "x": xs, "y": ys, "target": ts})
print(df)
df.to_csv("bo_evals.csv", index=False)
print("Constrained optimum (analytical): (x=2, y=1), f =", z_star_c)
# ---------- end ----------
