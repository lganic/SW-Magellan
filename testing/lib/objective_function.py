import math
import autograd.numpy as np
from typing import List, Dict
from .vectors import StateVector, ParameterVector
from .sim import sim
from .obstacle import Obstacle

def objective_function(
        starting_state: StateVector,
        target_state: StateVector, 
        params: ParameterVector, 
        obstacles: List[Obstacle],
        N: int,
        engine_thrust: float,
        starting_mass: float,
        fuel_consumption_rate: float,
        fuel_density: float,
        tuning_constants: Dict[str, float]):

    total_penalty = 0

    # First term is just TF
    total_penalty += params.Tf

    # Second term is the distance^2 from the end position to the target position.
    # For this we need the simulated path, so lets simulate it

    states = sim(starting_state, params, engine_thrust, starting_mass, fuel_consumption_rate, fuel_density)

    final_state = states[-1]

    # Now we can calculate the final position difference
    total_penalty += tuning_constants["Lambda P"] *(math.pow(final_state.x - target_state.x, 2) + math.pow(final_state.y - target_state.y, 2))

    # We can also add the penalty for the final velocity difference
    total_penalty += tuning_constants["Lambda V"] *(math.pow(final_state.vx - target_state.vx, 2) + math.pow(final_state.vy - target_state.vy, 2))

    # Now we need to add the smoothness penalty.
    # This is just the sum of the angle differences between each frame divided by delta T

    old_theta: float = None
    
    for new_theta in params.theta_n:

        if old_theta is not None:

            angle_delta = old_theta - new_theta

            angle_delta /= params.Delta_t

            angle_delta **= 2 # squaring

            total_penalty += tuning_constants["Lambda Mu"] * angle_delta

    # Now we just need to add in the penalty terms for the obstacles.

    for state in states:

        for obstacle in obstacles:

            total_penalty += obstacle.get_penalty(state)

    return total_penalty

def pack_params(params: ParameterVector) -> np.ndarray:
    return np.concatenate([
        np.array(params.theta_n),
        np.array(params.tau_n),
        np.array([params.Mu]),
    ])


def unpack_params(x: np.ndarray, N: int) -> ParameterVector:
    params = ParameterVector(N)

    params.theta_n = x[:N]
    params.tau_n = x[N:2*N]

    # Keep tau bounded smoothly-ish through optimizer bounds instead
    params.Mu = x[-1]

    return params

def make_autograd_objective(
    starting_state,
    target_state,
    obstacles,
    N,
    engine_thrust,
    starting_mass,
    fuel_consumption_rate,
    fuel_density,
    tuning_constants,
):
    def f(x):
        params = unpack_params(x, N)

        total_penalty = 0.0

        # Tf = exp(Mu), assuming your ParameterVector does this already.
        # If not, use:
        # params.Tf = np.exp(params.Mu)
        total_penalty = total_penalty + params.Tf

        states = sim(
            starting_state,
            params,
            engine_thrust,
            starting_mass,
            fuel_consumption_rate,
            fuel_density,
        )

        final_state = states[-1]

        total_penalty = total_penalty + tuning_constants["Lambda P"] * (
            (final_state.x - target_state.x) ** 2
            + (final_state.y - target_state.y) ** 2
        )

        total_penalty = total_penalty + tuning_constants["Lambda V"] * (
            (final_state.vx - target_state.vx) ** 2
            + (final_state.vy - target_state.vy) ** 2
        )

        for n in range(1, N):
            angle_delta = params.theta_n[n - 1] - params.theta_n[n]
            angle_rate = angle_delta / params.Delta_t
            total_penalty = total_penalty + tuning_constants["Lambda Mu"] * angle_rate ** 2

        for state in states:
            for obstacle in obstacles:
                total_penalty = total_penalty + obstacle.get_penalty(state)

        return total_penalty

    return f