import math
from typing import List
from .vectors import StateVector, ParameterVector
from .sim import sim
from .partials import get_drag_partials, get_tf_partial
from .obstacle import Obstacle
from .gradient import get_gradients
from matplotlib import pyplot as plt
import keyboard

def calculate_trajectory(
    starting_state: StateVector,
    end_state: StateVector,
    obstacles: List[Obstacle],
    N: int,
    engine_thrust: float,
    starting_mass: float,
    fuel_consumption_rate: float,
    fuel_density: float,
):
    params = ParameterVector(N)
    final_condition = False

    # --- live preview setup ---
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 6))

    trajectory_line, = ax.plot([], [], linewidth=2, label="Trajectory")
    quiver = None

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Trajectory with Control Vectors")
    ax.grid(True)
    ax.set_aspect("equal", adjustable="box")
    ax.legend()

    constants = {}

    constants["Lambda B"] = 4e-11
    constants["Lambda B"] = 4e-9
    constants["Lambda Mu"] = 1e-3
    constants["Lambda P"] = 1e-2
    constants["Lambda V"] = 3e2
    constants["Alpha Theta"] = 5e-8
    constants["Alpha Tau"] = 1e-10
    constants["Alpha Mu"] = 1e-10

    iteration = 0

    obstacle_patches = []

    while not final_condition:

        final_condition = keyboard.is_pressed('q')

        iteration += 1

        gradient = get_gradients(
            starting_state = starting_state,
            target_state = end_state,
            params = params,
            obstacles = obstacles,
            N = N,
            engine_thrust = engine_thrust,
            starting_mass = starting_mass,
            fuel_consumption_rate = fuel_consumption_rate,
            fuel_density = fuel_density,
            tuning_constants = constants
        )


        for n in range(N):
            params.theta_n[n] -= constants["Alpha Theta"] * gradient[n]
            params.tau_n[n] = min(1.0, max(0.0, params.tau_n[n] - constants["Alpha Tau"] * gradient[N + n]))

        params.Mu -= constants["Alpha Mu"] * gradient[-1]

        # This state check is purely for the graph, it is not needed for gradients
        states = sim(starting_state, params, engine_thrust, starting_mass, fuel_consumption_rate, fuel_density)

        # --- update live preview ---
        x = [s.x for s in states]
        y = [s.y for s in states]

        theta = params.theta_n
        tau = params.tau_n

        u = [tau[n] * math.cos(theta[n]) for n in range(len(theta))]
        v = [tau[n] * math.sin(theta[n]) for n in range(len(theta))]

        x_q = x[:-1]
        y_q = y[:-1]

        def s(k):

            for i in range(len(k)):
                if i % 10 != 0:
                    continue

                yield k[i]

        trajectory_line.set_data(x, y)

        if quiver is not None:
            quiver.remove()

        for p in obstacle_patches:
            p.remove()
        obstacle_patches.clear()

        quiver = ax.quiver(
            list(s(x_q)),
            list(s(y_q)),
            list(s(u)),
            list(s(v)),
            angles='xy',
            scale_units='xy',
            scale=.00001
        )

        # auto-rescale axes
        if x and y:

            min_x = min(x)
            max_x = max(x)

            min_y = min(y)
            max_y = max(y)

            pad_x = max(1.0, 0.05 * max(1.0, max_x - min_x))
            pad_y = max(1.0, 0.05 * max(1.0, max_y - min_y))
            ax.set_xlim(min_x - pad_x, max_x + pad_x)
            ax.set_ylim(min_y - pad_y, max_y + pad_y)

            for o in obstacles:
                patch = o.get_patch(min_x - pad_x, max_x + pad_x, min_y - pad_y, max_y + pad_y)
                ax.add_patch(patch)
                obstacle_patches.append(patch)
        
        ax.set_title(f"Trajectory with Control Vectors (iteration {iteration})")
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)

    plt.ioff()
    plt.show()