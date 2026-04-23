import math
from typing import List
from .vectors import StateVector, ParameterVector
from .sim import sim
from .partials import get_drag_partials
from .obstacle import Obstacle
from matplotlib import pyplot as plt

def _sub_m2(a, b):
    return 2 * (a - b)

def multiply_column_vec(state_gradient, jacobian):

    result = [0, 0, 0, 0]

    for j in range(4):
        sum = 0
        for i in range(4):
            sum += state_gradient[i] * jacobian[i][j]

        result[j] = sum

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
    # If mass at time n does NOT depend on tau_n, just return engine_thrust / mass
    if not include_same_step_mass_dependence:
        return engine_thrust / mass

    # Exact derivative if M_n includes fuel burned through step n
    dM_dtau = -fuel_density * fuel_consumption_rate * delta_t
    return (engine_thrust / mass) - (tau * engine_thrust / (mass * mass)) * dM_dtau
    # since dM_dtau is negative, this becomes +

def calculate_trajectory(starting_state: StateVector, end_state: StateVector, obstacles: List[Obstacle], N: int, engine_thrust: float, starting_mass: float, fuel_consumption_rate: float, fuel_density: float):

    # First, create our parameters which we will be using.
    params = ParameterVector(N)

    final_condition = False # Not sure what this will be yet. Change later.

    count = 0

    while not final_condition:

        # First, we need to forward simulate the whole path, given the parameter vector.

        states = sim(
            starting_state = starting_state,
            params = params,
            engine_thrust = engine_thrust,
            starting_mass = starting_mass,
            fuel_consumption_rate = fuel_consumption_rate,
            fuel_density = fuel_density
        )

        # Now that we have our forward pass states, we can calculate the gradient with respect to the terminal loss. This is just lambda_N from the paper

        final_state = states[-1]

        state_gradient = [
            _sub_m2(final_state.x, end_state.x),
            _sub_m2(final_state.y, end_state.y),
            _sub_m2(final_state.vx, end_state.vx),
            _sub_m2(final_state.vy, end_state.vy)
        ]

        # Now, we can backpropqgate this final state back through the gradient, and get each lambda n on the way.

        lambdas = [None] * (N + 1)
        lambdas[N] = state_gradient.copy()

        for n in range(N - 1, -1, -1):

            # For this, we need the partial of the boundary terms with respect to the state, and the jacobian.
            # First, we multiply the current state gradient by the jacobian

            state_gradient = multiply_column_vec(state_gradient, make_jacobian(states[n], params.Delta_t))

            # Now we add in all the penalty terms.

            for obstacle in obstacles:

                gradient = obstacle.get_gradient(states[n])

                state_gradient[0] += gradient[0]
                state_gradient[1] += gradient[1]

            lambdas[n] = state_gradient.copy()

        # Now that we have the lambda n backpropped, we have to convert these to our partials with respect to the controls.
        # This is gonna be that partial J with respect to zn = partial L with respect to zn + lambda n+1 partial s_n+1 with respect to zn
        grad_theta = [0.0] * N
        grad_tau = [0.0] * N

        # You need masses per step. Best: have sim() return them, or recompute exactly the same way.
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

        # add smoothness gradient
        lambda_mu = 1e-3  # or whatever holds this weight

        # assumes smoothness sum over n=1..N-1 of ((theta_n - theta_{n-1})**2)/delta_t
        for n in range(N):
            if n > 0:
                grad_theta[n] += 2 * lambda_mu * (params.theta_n[n] - params.theta_n[n - 1]) / ((params.Delta_t) ** 2)
            if n < N - 1:
                grad_theta[n] -= 2 * lambda_mu * (params.theta_n[n + 1] - params.theta_n[n]) / ((params.Delta_t) ** 2)

        alpha_theta = 1e-6
        alpha_tau = 1e-8

        for n in range(N):
            params.theta_n[n] -= alpha_theta * grad_theta[n]
            params.tau_n[n] = min(1.0, max(0.0, params.tau_n[n] - alpha_tau * grad_tau[n]))

        count += 1


        if count == 1000:

            x = [s.x for s in states]
            y = [s.y for s in states]

            # Use controls for quiver
            theta = params.theta_n
            tau = params.tau_n

            # Convert (theta, tau) -> vector components
            u = [tau[n] * math.cos(theta[n]) for n in range(len(theta))]
            v = [tau[n] * math.sin(theta[n]) for n in range(len(theta))]

            # Match sizes (states is N+1, controls is N)
            x_q = x[:-1]
            y_q = y[:-1]

            plt.figure(figsize=(8, 6))
            plt.plot(x, y, label="Trajectory", linewidth=2)

            plt.quiver(
                x_q,
                y_q,
                u,
                v,
                angles='xy',
                scale_units='xy',
                scale=1,
                color='red',
                label="Control (theta, tau)"
            )

            plt.xlabel("x")
            plt.ylabel("y")
            plt.title("Trajectory with Control Vectors")
            plt.axis('equal')
            plt.grid(True)
            plt.legend()

            plt.show()

            break