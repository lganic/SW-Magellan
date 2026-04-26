import math
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