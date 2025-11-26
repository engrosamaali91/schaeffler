#!/usr/bin/env python3

import csv
from pathlib import Path

import matplotlib.pyplot as plt

THIS_DIR = Path(__file__).resolve().parent
CSV_PATH = THIS_DIR / "bo_evals.csv"


def load_evals(csv_path: Path):
    iters = []
    max_vels = []
    acc_lims = []
    Js = []

    with csv_path.open("r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            iters.append(int(row["iter"]))
            max_vels.append(float(row["max_vel_x"]))
            acc_lims.append(float(row["acc_lim_x"]))
            Js.append(float(row["J"]))

    return iters, max_vels, acc_lims, Js


def main():
    if not CSV_PATH.exists():
        raise FileNotFoundError(f"{CSV_PATH} not found")

    iters, max_vels, acc_lims, Js = load_evals(CSV_PATH)

    # --- Find best J (min) ---
    min_J = min(Js)
    best_idx = Js.index(min_J)
    best_iter = iters[best_idx]
    best_v = max_vels[best_idx]
    best_a = acc_lims[best_idx]

    print("=== BO EVAL SUMMARY ===")
    print(f"Total iterations: {len(iters)}")
    print(f"Best J = {min_J:.6f} at iter={best_iter}")
    print(f"  max_vel_x = {best_v:.6f}")
    print(f"  acc_lim_x = {best_a:.6f}")
    print("========================")

    # --- Single overlapped plot with twin y-axes ---
    fig, ax1 = plt.subplots()

    # Left y-axis: J
    line_J, = ax1.plot(iters, Js, marker="o", linestyle="-", label="J (Sim2Real error)")
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("J (Sim2Real error)")
    ax1.grid(True, which="both", linestyle="--", alpha=0.4)

    # Mark best J point
    ax1.scatter([best_iter], [min_J], s=90, marker="*", color="red", zorder=5)
    ax1.annotate(
        f"Best J={min_J:.2f}\n(v={best_v:.3f}, a={best_a:.3f})",
        xy=(best_iter, min_J),
        xytext=(best_iter + 0.2, min_J * 1.05),
        arrowprops=dict(arrowstyle="->", lw=1),
        fontsize=9,
    )

    # Vertical line at best iteration
    ax1.axvline(best_iter, color="gray", linestyle=":", alpha=0.6)

    # Right y-axis: parameters
    ax2 = ax1.twinx()
    line_v, = ax2.plot(
        iters, max_vels, marker="s", linestyle="--", label="max_vel_x [m/s]"
    )
    line_a, = ax2.plot(
        iters, acc_lims, marker="d", linestyle="-.", label="acc_lim_x [m/s²]"
    )
    ax2.set_ylabel("Controller Parameters")

    # Combined legend from both axes
    lines = [line_J, line_v, line_a]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc="upper right")

    plt.title("Bayesian Optimization Progress (J, max_vel_x, acc_lim_x)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
