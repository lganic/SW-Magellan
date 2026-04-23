import math
from typing import Tuple
from .vectors import StateVector

class Obstacle:

    def get_penalty(self, state: StateVector) -> float:

        raise NotImplementedError("Inherited method not overridden.")
    
    def get_gradient(self, state: StateVector) -> Tuple[float, float]:

        raise NotImplementedError("Inherited method not overridden.")
    
class Ground(Obstacle):

    def __init__(self):
        pass

    def get_penalty(self, state: StateVector):
        
        return math.pow(max(0, -state.y), 2)
    
    def get_gradient(self, state: StateVector):

        if state.y >= 0:
            return (0.0, 0.0)

        return (0.0, -2 * state.y)