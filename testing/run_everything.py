import math
from lib.vectors import StateVector
from lib.descent_cg import (
    calculate_trajectory,
    fletcher_reeves,
    polak_ribiere,
    hestenes_stiefel,
)
from lib.descent_steepest import calculate_trajectory as calculate_trajectory_steepest
from lib.obstacle import (
    Ground,
    RightBasicIntersect,
    LeftBasicIntersect,
    BothBasicIntersect,
    LeftComplexIntersect,
    RightComplexIntersect,
    BothComplexIntersect,
    EllipseIntersect,
)
from matplotlib import pyplot as plt

def err_dist(p1: StateVector, p2: StateVector):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def err_vel(p1: StateVector, p2: StateVector):
    return math.sqrt((p1.vx - p2.vx) ** 2 + (p1.vy - p2.vy) ** 2)


def run_trial(method_name, target_name, solver_func, sv, target, obstacles, cg_beta_func=None):
    if cg_beta_func is None:
        states, params, iteration, losses = solver_func(
            sv, target, obstacles, 500, 200000, 4500, 1, 1, name = method_name
        )
    else:
        states, params, iteration, losses = solver_func(
            sv, target, obstacles, 500, 200000, 4500, 1, 1, cg_beta_func, name = method_name
        )

    final_state = states[-1]

    return {
        "target": target_name,
        "method": method_name,
        "distance_error": err_dist(target, final_state),
        "velocity_error": err_vel(target, final_state),
        "Tf": params.Tf,
        "iterations": iteration,
        "final_loss": losses[-1],
        "losses": losses,
    }


def print_results_table(results):
    headers = [
        "Target",
        "Method",
        "Dist Error",
        "Vel Error",
        "Tf",
        "Iterations",
        "Final Loss",
    ]

    rows = [
        [
            r["target"],
            r["method"],
            f"{r['distance_error']:.3f}",
            f"{r['velocity_error']:.3f}",
            f"{r['Tf']:.3f}",
            str(r["iterations"]),
            f"{r['final_loss']:.6e}",
        ]
        for r in results
    ]

    col_widths = [
        max(len(headers[i]), *(len(row[i]) for row in rows))
        for i in range(len(headers))
    ]

    def format_row(row):
        return " | ".join(
            value.ljust(col_widths[i])
            for i, value in enumerate(row)
        )

    print(format_row(headers))
    print("-+-".join("-" * w for w in col_widths))

    for row in rows:
        print(format_row(row))


def plot_objective_losses(results):
    targets = sorted(set(r["target"] for r in results))

    for target in targets:
        plt.figure(figsize=(10, 6))

        target_results = [r for r in results if r["target"] == target]

        for r in target_results:
            plt.plot(
                range(len(r["losses"])),
                r["losses"],
                label=r["method"],
            )

        plt.title(f"Objective Loss Comparison - {target}")
        plt.xlabel("Iteration")
        plt.ylabel("Objective Loss")
        plt.yscale("log")  # useful if losses vary wildly
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    plt.show()

sv = StateVector(0, 0, 0, 500)

targets = {
    "Moon": StateVector(200000, 80000, 0, 0),
    "Orbit": StateVector(20000, 300000, 0, 0),
}

obstacles = [
    Ground(),
    LeftBasicIntersect(-128000, 40000),
    LeftComplexIntersect(-40000, 128000, 40000),
    BothComplexIntersect(40000, 128000, 128000, 40000),
    BothBasicIntersect(128000, 184000, 128000),
    RightBasicIntersect(216000, 128000),
    EllipseIntersect(100000, 128000, 60000, 60000),
]

methods = [
    ("Steepest", calculate_trajectory_steepest, None),
    ("Fletcher-Reeves", calculate_trajectory, fletcher_reeves),
    ("Polak-Ribiere", calculate_trajectory, polak_ribiere),
    ("Hestenes-Stiefel", calculate_trajectory, hestenes_stiefel),
]

results = []

for target_name, target in targets.items():
    for method_name, solver_func, beta_func in methods:
        result = run_trial(
            method_name,
            target_name,
            solver_func,
            sv,
            target,
            obstacles,
            cg_beta_func=beta_func,
        )

        results.append(result)

print_results_table(results)
plot_objective_losses(results)