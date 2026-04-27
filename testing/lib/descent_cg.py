import math
from typing import List
from .vectors import StateVector, ParameterVector
from .sim import sim
from .obstacle import Obstacle
from .gradient import get_gradients
from .objective_function import objective_function
from .non_gradient_optimizer import nelder_mead_1d
from .golden_search import golden_search
from matplotlib import pyplot as plt
import keyboard

def neg(vec):
    return [-a for a in vec]

def add(vec_a, vec_b):

    return [a + b for a, b in zip(vec_a, vec_b)]

def sub(vec_a, vec_b):

    return [a - b for a, b in zip(vec_a, vec_b)]

def scale(vec, scalar):

    return [a * scalar for a in vec]

def dot(vec_a, vec_b):

    return sum([a * b for a, b in zip(vec_a, vec_b)])

def gradient_magnitude(gradient):

    # Just dot product of the gradient with itself (since l2 norm squared), so:
    return dot(gradient, gradient)

def fletcher_reeves(current_gradient, old_gradient, _):

    return gradient_magnitude(current_gradient) / gradient_magnitude(old_gradient)

def polak_ribiere(current_gradient, old_gradient, _):

    return dot(current_gradient, sub(current_gradient, old_gradient)) / gradient_magnitude(old_gradient)

def hestenes_stiefel(current_gradient, old_gradient, descent_direction):

    return dot(current_gradient, sub(current_gradient, old_gradient)) / (dot(descent_direction, sub(current_gradient, old_gradient)))

def calculate_trajectory(
    starting_state: StateVector,
    end_state: StateVector,
    obstacles: List[Obstacle],
    N: int,
    engine_thrust: float,
    starting_mass: float,
    fuel_consumption_rate: float,
    fuel_density: float,
    beta_function
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

    all_objective_losses = []

    prev_gradient = None
    p_direction = None

    while not final_condition:

        final_condition = keyboard.is_pressed('q')

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

        # Multiply gradient by tuning values. 

        # Multiply theta and tau
        for i in range(N):
            gradient[i] = constants["Alpha Theta"] * gradient[i]
            gradient[N + i] = constants["Alpha Tau"] * gradient[N + i]

        gradient[-1] = constants["Alpha Mu"] * gradient[-1]

        if iteration == 0:
            # First iteration. p is just the gradient.
            p_direction = neg(gradient)
        else:
            beta = beta_function(gradient, prev_gradient, p_direction)
            beta = max(0.0, beta)

            candidate_direction = add(
                neg(gradient),
                scale(p_direction, beta)
            )

            if dot(candidate_direction, gradient) >= 0:
                print("CG direction was not descent. Resetting direction")
                p_direction = neg(gradient)
            else:
                p_direction = candidate_direction

        prev_gradient = gradient.copy()

        # Now we have our search direction, we need to find the optimal alpha to use when determining the next state. 
        # For this, i am going to use a golden search, so we need a function which takes in an alpha, and returns the objective at that alpha

        def search_function(alpha):

            # We need to create our offset parameter vector, using the update function xk+1=xk+ak*pk+1
            new_params = add(params.pack(), scale(p_direction, alpha))

            new_param_object = ParameterVector.unpack(new_params)

            result = objective_function(starting_state, end_state, new_param_object, obstacles, N, engine_thrust, starting_mass, fuel_consumption_rate, fuel_density, constants)

            return result
        
        # Now to find the best alpha, we just call golden search
        alpha = golden_search(search_function, .1, 1, 1e-5) # Tuning these, as sometimes with too high an alpha it likes to jump through certain boundaries it really shouldn't. Too low an alpha and it doesn't go anywhere.

        # Now our alpha is the optimal amount to adjust by.

        new_params = add(params.pack(), scale(p_direction, alpha))

        params = ParameterVector.unpack(new_params)

        objective = objective_function(starting_state, end_state, params, obstacles, N, engine_thrust, starting_mass, fuel_consumption_rate, fuel_density, constants)

        all_objective_losses.append(objective)
        print("Objective: ", objective)

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

        iteration += 1

    plt.ioff()
    plt.show()

    plt.figure()
    plt.plot(all_objective_losses)
    plt.title("Objective Function vs Iteration")
    plt.show()

    return params, iteration, all_objective_losses