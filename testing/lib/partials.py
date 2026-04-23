import math
from .vectors import StateVector

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