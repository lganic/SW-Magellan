import math
from typing import List, Dict
from .vectors import StateVector, ParameterVector
from .sim import sim
from .partials import get_drag_partials, get_tf_partial
from .obstacle import Obstacle

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
):
    dM_dtau = -fuel_density * fuel_consumption_rate * delta_t
    return (engine_thrust / mass) - (tau * engine_thrust / (mass * mass)) * dM_dtau

def get_gradients(
    starting_state: StateVector,
    target_state: StateVector, 
    params: ParameterVector, 
    obstacles: List[Obstacle],
    N: int,
    engine_thrust: float,
    starting_mass: float,
    fuel_consumption_rate: float,
    fuel_density: float,
    tuning_constants: Dict[str, float]) -> List[float]:

    # First forward simulate the params so we have something to backprop over
    states = sim(
        starting_state=starting_state,
        params=params,
        engine_thrust=engine_thrust,
        starting_mass=starting_mass,
        fuel_consumption_rate=fuel_consumption_rate,
        fuel_density=fuel_density
    )

    final_state = states[-1]

    # Get the final lambda N which we will be using for backprop
    state_gradient = [
        tuning_constants["Lambda P"] * _sub_m2(final_state.x, target_state.x),
        tuning_constants["Lambda P"] * _sub_m2(final_state.y, target_state.y),
        tuning_constants["Lambda V"] * _sub_m2(final_state.vx, target_state.vx),
        tuning_constants["Lambda V"] * _sub_m2(final_state.vy, target_state.vy)
    ]

    # Build out an output space so we can avoid some weird indexing issues
    lambdas = [None] * (N + 1)
    lambdas[N] = state_gradient.copy()

    for n in range(N - 1, -1, -1):

        # multiply by partial sn+1 / sn
        state_gradient = multiply_column_vec(
            state_gradient,
            make_jacobian(states[n], params.Delta_t)
        )

        # Add in the other part of the recursion backward (partial L / sn)
        # Smoothness drops out, so we only have the boundary gradient
        for obstacle in obstacles:
            gradient = obstacle.get_gradient(states[n])

            state_gradient[0] += tuning_constants["Lambda B"] * gradient[0]
            state_gradient[1] += tuning_constants["Lambda B"] * gradient[1]

        lambdas[n] = state_gradient.copy()

    # Now that we have our adjoints, we need to calculate the partial with respect to the control vector.
    grad_theta = [0.0] * N
    grad_tau = [0.0] * N

    masses = [state.mass for state in states]

    for n in range(N):

        # This is where we are calculating the partial of the next state with respect to the control vector.
        theta = params.theta_n[n]
        tau = params.tau_n[n]
        mass = masses[n]

        # # Get the acceleration due to the thrust as this stage. 
        a = thrust_accel(tau, mass, engine_thrust)

        # The only parts of the next state which depend on theta is the thrust, so this is the final version of this value.
        ds_dtheta = [
            0.0,
            0.0,
            params.Delta_t * (-a * math.sin(theta)),
            params.Delta_t * ( a * math.cos(theta)),
        ]

        # This derivative is a little more complex, so I isolated it to its own function
        da_dtau = dthrust_dtau(
            tau=tau,
            mass=mass,
            engine_thrust=engine_thrust,
            delta_t=params.Delta_t,
            fuel_consumption_rate=fuel_consumption_rate,
            fuel_density=fuel_density,
        )

        # Build out the derivative with respect to tau vector
        ds_dtau = [
            0.0,
            0.0,
            params.Delta_t * math.cos(theta) * da_dtau,
            params.Delta_t * math.sin(theta) * da_dtau,
        ]

        # Multiply by lambda, and add to gradient.
        grad_theta[n] += dot4(lambdas[n + 1], ds_dtheta)
        grad_tau[n] += dot4(lambdas[n + 1], ds_dtau)

    # Add in the smoothness partial
    for n in range(N):
        if n > 0:
            grad_theta[n] += 2 * tuning_constants["Lambda Mu"] * (params.theta_n[n] - params.theta_n[n - 1]) / (params.Delta_t ** 2)
        if n < N - 1:
            grad_theta[n] -= 2 * tuning_constants["Lambda Mu"] * (params.theta_n[n + 1] - params.theta_n[n]) / (params.Delta_t ** 2)

    # Get the gradient with respect to mu (which directly controls Tf via Tf=e^mu)
    grad_Tf = get_tf_partial(
        states,
        params=params,
        lambdas = lambdas,
        engine_thrust=engine_thrust,
        starting_mass=starting_mass,
        fuel_consumption_rate=fuel_consumption_rate,
        fuel_density=fuel_density
    )

    # Add the smoothing to the mu partial (smoothness depends on mu, since we divide by delta T)
    smooth_sum = 0.0

    for n in range(1, N):
        dtheta = params.theta_n[n] - params.theta_n[n - 1]
        smooth_sum += dtheta * dtheta

    grad_Tf += tuning_constants["Lambda Mu"] * (-2.0 * smooth_sum / (params.Delta_t ** 3)) * (1.0 / N)

    # Multiply by Tf, to fulfill the partial conversion. (partial J / partial mu = partial J / partial Tf * partial tf / partial mu & partial Tf / partial mu = e^mu = tf) So we just have to multiply by Tf
    grad_mu = params.Tf * grad_Tf

    # Return packed gradient vector
    return grad_theta + grad_tau + [grad_mu]