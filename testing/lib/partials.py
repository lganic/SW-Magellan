import math
from typing import List
from .vectors import StateVector, ParameterVector
from .forces import drag, field

def get_drag_partials(state: StateVector):
    v = state.v
    vx = state.vx
    vy = state.vy

    if v < 500 or v < 1e-12:
        return [0.0, 0.0, 0.0, 0.0]

    # Subgradient has activated. Gotta do the math.
    # We are gonna go xx, xy, yx, yy

    c = 1000.0 / (3.0 * v**3)

    dax_dvx = -2.0 / 3.0 + c * vy * vy
    dax_dvy = -c * vx * vy
    day_dvx = -c * vx * vy
    day_dvy = -2.0 / 3.0 + c * vx * vx

    return [dax_dvx, dax_dvy, day_dvx, day_dvy]

def dot4(a, b):
    return sum(x * y for x, y in zip(a, b))

def get_tf_partial(starting_state: StateVector, params: ParameterVector, lambdas: List[float], engine_thrust: float, starting_mass: float, fuel_consumption_rate: float, fuel_density: float):

    delta_t = params.Delta_t

    current_state = starting_state
    current_mass = starting_mass

    starting_state.mass = starting_mass

    grad_Tf = 0

    tau_prefix_sum = 0

    for n, (theta, throttle) in enumerate(zip(params.theta_n, params.tau_n)):

        thrust_mag = throttle * engine_thrust / current_mass

        tau_prefix_sum += throttle

        a_x_thrust = thrust_mag * math.cos(theta)
        a_y_thrust = thrust_mag * math.sin(theta)

        a_x_field, a_y_field = field(current_state)

        a_x_drag, a_y_drag = drag(current_state)

        new_x = current_state.x + delta_t * current_state.vx
        new_y = current_state.y + delta_t * current_state.vy

        ax = a_x_thrust + a_x_drag + a_x_field
        ay = a_y_thrust + a_y_drag + a_y_field

        new_vx = current_state.vx + delta_t * ax
        new_vy = current_state.vy + delta_t * ay

        current_state = StateVector(new_x, new_y, new_vx, new_vy)

        current_state.mass = current_mass

        current_mass -= fuel_consumption_rate * fuel_density * delta_t * throttle

        # dM_n / dDelta_t
        dM_dDelta_t = -fuel_density * fuel_consumption_rate * tau_prefix_sum

        # d(a_thrust) / dDelta_t
        da_thrust_dDelta_t = -(throttle * engine_thrust / (current_mass * current_mass)) * dM_dDelta_t

        ds_dDelta_t = [
            current_state.vx,
            current_state.vy,
            ax + params.Delta_t * math.cos(theta) * da_thrust_dDelta_t,
            ay + params.Delta_t * math.sin(theta) * da_thrust_dDelta_t,
        ]

        ds_dTf = [v / params.N for v in ds_dDelta_t]

        grad_Tf += dot4(lambdas[n + 1], ds_dTf)

    return grad_Tf