import math
from typing import List
from .vectors import StateVector, ParameterVector
from .sim import sim
from .partials import get_drag_partials, get_tf_partial
from .obstacle import Obstacle
from matplotlib import pyplot as plt
import keyboard

def _sub_m2(a, b):
    return 2 * (a - b)

def multiply_column_vec(state_gradient, jacobian):
    result = [0, 0, 0, 0]

    for j in range(4):
        total = 0
        for i in range(4):
            total += state_gradient[i] * jacobian[i][j]
        result[j] = total

    return result

def make_jacobian(state: StateVector, delta_t: float):
    drag_partials = get_drag_partials(state)

    g = 10
    R = (100000 / 3) * (1 + math.sqrt(10))

    return [
        [1, 0, delta_t, 0],
        [0, 1, 0, delta_t],
        [0, 0, 1 + delta_t * drag_partials[0], delta_t * drag_partials[1]],
        [0, delta_t * (2 * g * R * R / math.pow(state.y + R, 3)), delta_t * (1e-2 + drag_partials[2]), 1 + delta_t * drag_partials[3]],
    ]

def dot4(a, b):
    return sum(x * y for x, y in zip(a, b))

def thrust_accel(tau, mass, engine_thrust):

    return tau * engine_thrust / mass

def dthrust_dtau(
    tau: float,
    mass: float,
    engine_thrust: float,
    delta_t: float,
    fuel_consumption_rate: float,
    fuel_density: float,
    include_same_step_mass_dependence: bool = True,
):
    if not include_same_step_mass_dependence:
        return engine_thrust / mass

    dM_dtau = -fuel_density * fuel_consumption_rate * delta_t
    return (engine_thrust / mass) - (tau * engine_thrust / (mass * mass)) * dM_dtau

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
    ax.axis("equal")
    ax.legend()

    lambda_b = 1e-3

    iteration = 0

    while not final_condition:

        final_condition = keyboard.is_pressed('q')

        iteration += 1

        states = sim(
            starting_state=starting_state,
            params=params,
            engine_thrust=engine_thrust,
            starting_mass=starting_mass,
            fuel_consumption_rate=fuel_consumption_rate,
            fuel_density=fuel_density
        )

        final_state = states[-1]

        state_gradient = [
            _sub_m2(final_state.x, end_state.x),
            _sub_m2(final_state.y, end_state.y),
            _sub_m2(final_state.vx, end_state.vx),
            _sub_m2(final_state.vy, end_state.vy)
        ]

        lambdas = [None] * (N + 1)
        lambdas[N] = state_gradient.copy()

        for n in range(N - 1, -1, -1):
            state_gradient = multiply_column_vec(
                state_gradient,
                make_jacobian(states[n], params.Delta_t)
            )

            for obstacle in obstacles:
                gradient = obstacle.get_gradient(states[n])
                state_gradient[0] += lambda_b * gradient[0]
                state_gradient[1] += lambda_b * gradient[1]

            lambdas[n] = state_gradient.copy()

        grad_theta = [0.0] * N
        grad_tau = [0.0] * N

        masses = [state.mass for state in states]

        for n in range(N):
            theta = params.theta_n[n]
            tau = params.tau_n[n]
            mass = masses[n]

            a = thrust_accel(tau, mass, engine_thrust)

            ds_dtheta = [
                0.0,
                0.0,
                params.Delta_t * (-a * math.sin(theta)),
                params.Delta_t * ( a * math.cos(theta)),
            ]

            da_dtau = dthrust_dtau(
                tau=tau,
                mass=mass,
                engine_thrust=engine_thrust,
                delta_t=params.Delta_t,
                fuel_consumption_rate=fuel_consumption_rate,
                fuel_density=fuel_density,
                include_same_step_mass_dependence=True,
            )

            ds_dtau = [
                0.0,
                0.0,
                params.Delta_t * math.cos(theta) * da_dtau,
                params.Delta_t * math.sin(theta) * da_dtau,
            ]

            grad_theta[n] += dot4(lambdas[n + 1], ds_dtheta)
            grad_tau[n] += dot4(lambdas[n + 1], ds_dtau)

        lambda_mu = 1e-4

        for n in range(N):
            if n > 0:
                grad_theta[n] += 2 * lambda_mu * (params.theta_n[n] - params.theta_n[n - 1]) / (params.Delta_t ** 2)
            if n < N - 1:
                grad_theta[n] -= 2 * lambda_mu * (params.theta_n[n + 1] - params.theta_n[n]) / (params.Delta_t ** 2)

        alpha_theta = 1e-8
        alpha_tau = 1e-15
        alpha_mu = 1e-13

        grad_Tf = get_tf_partial(
            starting_state=starting_state,
            params=params,
            lambdas = lambdas,
            engine_thrust=engine_thrust,
            starting_mass=starting_mass,
            fuel_consumption_rate=fuel_consumption_rate,
            fuel_density=fuel_density
        )

        for n in range(N):
            params.theta_n[n] -= alpha_theta * grad_theta[n]
            params.tau_n[n] = min(1.0, max(0.0, params.tau_n[n] - alpha_tau * grad_tau[n]))

        smooth_sum = 0.0

        for n in range(1, N):
            dtheta = params.theta_n[n] - params.theta_n[n - 1]
            smooth_sum += dtheta * dtheta

        grad_Tf += lambda_mu * (-2.0 * smooth_sum / (params.Delta_t ** 3)) * (1.0 / N)

        grad_mu = params.Tf * grad_Tf

        params.Mu -= alpha_mu * grad_mu

        # --- update live preview ---
        x = [s.x for s in states]
        y = [s.y for s in states]

        theta = params.theta_n
        tau = params.tau_n

        u = [tau[n] * math.cos(theta[n]) for n in range(len(theta))]
        v = [tau[n] * math.sin(theta[n]) for n in range(len(theta))]

        x_q = x[:-1]
        y_q = y[:-1]

        trajectory_line.set_data(x, y)

        if quiver is not None:
            quiver.remove()

        quiver = ax.quiver(
            x_q,
            y_q,
            u,
            v,
            angles='xy',
            scale_units='xy',
            scale=1
        )

        # auto-rescale axes
        if x and y:
            pad_x = max(1.0, 0.05 * max(1.0, max(x) - min(x)))
            pad_y = max(1.0, 0.05 * max(1.0, max(y) - min(y)))
            ax.set_xlim(min(x) - pad_x, max(x) + pad_x)
            ax.set_ylim(min(y) - pad_y, max(y) + pad_y)

        ax.set_title(f"Trajectory with Control Vectors (iteration {iteration})")
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)

        # TODO: replace with your real stopping condition
        # for example:
        # if iteration >= 500:
        #     final_condition = True

    plt.ioff()
    plt.show()