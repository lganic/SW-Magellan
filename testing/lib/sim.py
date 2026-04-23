'''
This file defines forward simulation logic.
'''

import math
from .vectors import StateVector, ParameterVector
from .forces import field, drag

def sim(starting_state: StateVector, params: ParameterVector, engine_thrust: float, starting_mass: float, fuel_consumption_rate: float, fuel_density: float):

    delta_t = params.Delta_t

    current_state = starting_state
    current_mass = starting_mass

    output = [starting_state.copy()]

    for theta, throttle in zip(params.theta_n, params.tau_n):


        thrust_mag = throttle * engine_thrust / current_mass

        a_x_thrust = thrust_mag * math.cos(theta)
        a_y_thrust = thrust_mag * math.sin(theta)

        a_x_field, a_y_field = field(current_state)

        a_x_drag, a_y_drag = drag(current_state)

        new_x = current_state.x + delta_t * current_state.vx
        new_y = current_state.y + delta_t * current_state.vy

        new_vx = current_state.vx + delta_t * (a_x_thrust + a_x_drag + a_x_field)
        new_vy = current_state.vy + delta_t * (a_y_thrust + a_y_drag + a_y_field)

        current_state = StateVector(new_x, new_y, new_vx, new_vy)

        output.append(current_state)

        current_mass -= fuel_consumption_rate * fuel_density * delta_t * throttle

    return output