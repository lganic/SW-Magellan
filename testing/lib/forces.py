'''
This file defines the forces which apply to the spacecraft during travel, mostly for forward simulation logic.
'''

import math
import autograd.numpy as np
from .vectors import StateVector

R = (100000 / 3) * (1 + math.sqrt(10))
g = 10

def field(sv: StateVector):
    
    # Simulate the force due to gravity in the Stormworks Physics model. 

    ground_velocity = sv.vx

    y = sv.y

    return (0, 1 + (np.abs(ground_velocity) / 100) - (g * R * R) / np.power(y + R, 2))

def drag(sv: StateVector):

    velocity = sv.v

    if velocity < 1e-4:
        return (0, 0)

    accel_mag = -np.maximum(0, (velocity - 500) / 1.5)

    return (accel_mag * sv.vx / velocity, accel_mag * sv.vy / velocity)