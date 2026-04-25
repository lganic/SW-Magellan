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

def get_tf_partial(states: List[StateVector], params: ParameterVector, lambdas: List[float], engine_thrust: float, starting_mass: float, fuel_consumption_rate: float, fuel_density: float):

    grad_Tf = 1 # From the Tf + ... Part of the jacobian

    tau_prefix_sum = 0

    old_velocity_x = 0
    old_velocity_y = 0

    for n, (theta, throttle) in enumerate(zip(params.theta_n, params.tau_n)):

        s = states[n]
        mass = s.mass

        thrust_mag = throttle * engine_thrust / mass

        a_x_thrust = thrust_mag * math.cos(theta)
        a_y_thrust = thrust_mag * math.sin(theta)

        a_x_field, a_y_field = field(s)

        a_x_drag, a_y_drag = drag(s)

        ax = a_x_thrust + a_x_drag + a_x_field
        ay = a_y_thrust + a_y_drag + a_y_field

        # dM_n / dDelta_t
        dM_dDelta_t = -fuel_density * fuel_consumption_rate * tau_prefix_sum

        # d(a_thrust) / dDelta_t
        da_thrust_dDelta_t = -(throttle * engine_thrust / (mass * mass)) * dM_dDelta_t

        ds_dDelta_t = [
            old_velocity_x,
            old_velocity_y,
            ax + params.Delta_t * math.cos(theta) * da_thrust_dDelta_t,
            ay + params.Delta_t * math.sin(theta) * da_thrust_dDelta_t,
        ]

        old_velocity_x = s.vx
        old_velocity_y = s.vy

        ds_dTf = [v / params.N for v in ds_dDelta_t]

        grad_Tf += dot4(lambdas[n + 1], ds_dTf)

        tau_prefix_sum += throttle

    return grad_Tf