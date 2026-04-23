'''
This file defines the state vector format, as well as the parameter vector format.
'''

import math

class StateVector:

    def __init__(self, x, y, vx, vy):

        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy

    def copy(self):

        return StateVector(self.x, self.y, self.vx, self.vy)
    
    def __repr__(self):

        return f'({self.x}, {self.y}, {self.vx}, {self.vy})'

class ParameterVector:

    def __init__(self, number_of_elements, starting_angle = math.pi / 2, starting_throttle = 1, starting_time = 60*60):

        '''
        Create a state vector with a given size.
        '''

        self.N = number_of_elements

        self.theta_n = [starting_angle] * number_of_elements
        self.tau_n = [starting_throttle] * number_of_elements

        # self.Tf = starting_time
        self.Mu = math.log(starting_time)

    @property
    def Tf(self):

        return math.pow(math.e, self.Mu)
    
    @property
    def Delta_t(self):

        return self.Tf / self.N