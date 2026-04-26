'''
This file defines the state vector format, as well as the parameter vector format.
'''

import math
import autograd.numpy as np

class StateVector:

    def __init__(self, x, y, vx, vy):

        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy

        self.mass = None

    def copy(self):

        out = StateVector(self.x, self.y, self.vx, self.vy)

        out.mass = self.mass

        return out
    
    @property
    def v(self):
        # TODO: optimize
        return np.sqrt(np.power(self.vx, 2) + np.power(self.vy, 2))

    def __repr__(self):

        return f'({self.x}, {self.y}, {self.vx}, {self.vy})'

class ParameterVector:

    def __init__(self, number_of_elements, starting_angle = math.pi / 2, starting_throttle = 1, starting_time = 15 * 60):

        '''
        Create a state vector with a given size.
        '''

        self.N = number_of_elements

        self.theta_n = [starting_angle] * number_of_elements
        self.tau_n = [starting_throttle] * number_of_elements

        # self.Tf = starting_time
        self.Mu = np.log(starting_time)

    @property
    def Tf(self):

        try:
            return np.power(np.e, self.Mu)
        except OverflowError:
            print(self.Mu)
            raise OverflowError("Mu too big")


    
    @property
    def Delta_t(self):

        return self.Tf / self.N