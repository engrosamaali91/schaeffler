# #!/usr/bin/env python3

# import argparse
# import csv
# from pathlib import Path

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
# from scipy.interpolate import griddata

# THIS_DIR = Path(__file__).resolve().parent


# def load_evals(csv_path: Path):
#     iters = []
#     max_vels = []
#     acc_lims = []
#     Js = []

#     with csv_path.open("r") as f:
#         reader = csv.DictReader(f)
#         for row in reader:
#             iters.append(int(row["iter"]))
#             max_vels.append(float(row["max_vel_x"]))
#             acc_lims.append(float(row["acc_lim_x"]))
#             Js.append(float(row["J"]))

#     return np.array(iters), np.array(max_vels), np.array(acc_lims), np.array(Js)


# def main():
#     ap = argparse.ArgumentParser(description="Plot BO optimization surface from evaluation CSV")
#     ap.add_argument(
#         "--eval",
#         type=str,
#         default="bo_evals.csv",
#         help="Path to bo_evals.csv (default: bo_evals.csv in current directory)",
#     )
#     args = ap.parse_args()

#     # Resolve the CSV path
#     csv_path = Path(args.eval)
#     if not csv_path.is_absolute():
#         csv_path = THIS_DIR / csv_path

#     if not csv_path.exists():
#         raise FileNotFoundError(f"{csv_path} not found")

#     iters, max_vels, acc_lims, Js = load_evals(csv_path)

#     # --- Best point (min J) ---
#     best_idx = np.argmin(Js)
#     best_iter = iters[best_idx]
#     best_v = max_vels[best_idx]
#     best_a = acc_lims[best_idx]
#     best_J = Js[best_idx]

#     print("=== BO SURFACE SUMMARY ===")
#     print(f"Total iterations: {len(iters)}")
#     print(f"Best J = {best_J:.6f} at iter={best_iter}")
#     print(f"  max_vel_x = {best_v:.6f}")
#     print(f"  acc_lim_x = {best_a:.6f}")
#     print("===========================")

#     # --- Build a grid over the explored parameter region ---
#     v_min, v_max = max_vels.min(), max_vels.max()
#     a_min, a_max = acc_lims.min(), acc_lims.max()

#     # Slight padding so points are not exactly on the boundaries
#     pad_v = 0.02 * (v_max - v_min) if v_max > v_min else 0.01
#     pad_a = 0.02 * (a_max - a_min) if a_max > a_min else 0.01

#     v_lin = np.linspace(v_min - pad_v, v_max + pad_v, 60)
#     a_lin = np.linspace(a_min - pad_a, a_max + pad_a, 60)
#     V_grid, A_grid = np.meshgrid(v_lin, a_lin)

#     # Interpolate J over the grid (approximate unknown cost surface)
#     points = np.column_stack((max_vels, acc_lims))
#     J_grid = griddata(
#         points,
#         Js,
#         (V_grid, A_grid),
#         method="cubic",
#     )

#     # If cubic fails (few points), fall back to linear
#     if np.isnan(J_grid).all():
#         J_grid = griddata(
#             points,
#             Js,
#             (V_grid, A_grid),
#             method="linear",
#         )

#     # --- 3D surface plot ---
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection="3d")

#     surf = ax.plot_surface(
#         V_grid,
#         A_grid,
#         J_grid,
#         cmap="plasma",
#         alpha=0.85,
#         edgecolor="none",
#     )

#     # Sampled points (all BO evaluations)
#     ax.scatter(
#         max_vels,
#         acc_lims,
#         Js,
#         c="red",
#         s=35,
#         label="Sampled points",
#         depthshade=True,
#     )

#     # Highlight best point
#     ax.scatter(
#         [best_v],
#         [best_a],
#         [best_J],
#         c="red",
#         s=120,
#         marker="*",
#         label="Best J",
#         depthshade=True,
#     )

#     ax.set_xlabel("max_vel_x [m/s]")
#     ax.set_ylabel("acc_lim_x [m/s²]")
#     ax.set_zlabel("J (Sim2Real error)")
#     ax.set_title("Bayesian Optimization Cost Surface (interpolated)")

#     fig.colorbar(surf, shrink=0.6, aspect=15, label="J (interpolated)")

#     ax.legend(loc="upper left")
#     plt.tight_layout()
#     plt.show()


# if __name__ == "__main__":
#     main()


#code for the next two columns ...


import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import griddata

def load_evals(csv_path: Path):
    data = {
        "iter": [], "vx": [], "ax": [], 
        "vth": [], "ath": [], "J": []
    }
    with csv_path.open("r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data["iter"].append(int(row["iter"]))
            data["vx"].append(float(row["max_vel_x"]))
            data["ax"].append(float(row["acc_lim_x"]))
            data["vth"].append(float(row["max_vel_theta"]))
            data["ath"].append(float(row["acc_lim_theta"]))
            data["J"].append(float(row["J"]))
    return {k: np.array(v) for k, v in data.items()}

def create_surface(ax, x, y, z, xlabel, ylabel, title, best_pt):
    # Create grid
    xi = np.linspace(x.min(), x.max(), 50)
    yi = np.linspace(y.min(), y.max(), 50)
    X, Y = np.meshgrid(xi, yi)
    
    # Interpolate
    Z = griddata((x, y), z, (X, Y), method='cubic')
    if np.isnan(Z).all(): # Fallback
        Z = griddata((x, y), z, (X, Y), method='linear')

    # Plot surface
    surf = ax.plot_surface(X, Y, Z, cmap='plasma', alpha=0.8, edgecolor='none')
    
    # Plot sampled points
    ax.scatter(x, y, z, c='red', s=20)
    
    # Best point star
    ax.scatter(best_pt[0], best_pt[1], best_pt[2], c='red', marker='*', s=150, label='Best J')
    
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel('J')
    ax.set_title(title)
    return surf

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval", type=str, required=True)
    args = ap.parse_args()
    
    d = load_evals(Path(args.eval).expanduser())
    best_idx = np.argmin(d["J"])

    fig = plt.figure(figsize=(16, 8))

    # --- Plot 1: Linear Params ---
    ax1 = fig.add_subplot(121, projection='3d')
    surf1 = create_surface(ax1, d["vx"], d["ax"], d["J"], 
                           'max_vel_x', 'acc_lim_x', 'Linear Params vs J',
                           (d["vx"][best_idx], d["ax"][best_idx], d["J"][best_idx]))

    # --- Plot 2: Angular Params ---
    ax2 = fig.add_subplot(122, projection='3d')
    surf2 = create_surface(ax2, d["vth"], d["ath"], d["J"], 
                           'max_vel_theta', 'acc_lim_theta', 'Angular Params vs J',
                           (d["vth"][best_idx], d["ath"][best_idx], d["J"][best_idx]))

    fig.colorbar(surf1, ax=[ax1, ax2], shrink=0.5, label='J (Error)')
    plt.suptitle(f"Bayesian Optimization Analysis (Best J={d['J'][best_idx]:.3f} at Iter {d['iter'][best_idx]})")
    plt.show()

if __name__ == "__main__":
    main()