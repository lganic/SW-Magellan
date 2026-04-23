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

class ParameterVector:

    def __init__(self, number_of_elements, starting_angle = math.pi / 2, starting_throttle = 1, starting_time = 60*60):

        '''
        Create a state vector with a given size.
        '''

        self.N = number_of_elements

        self.theta_n = [starting_angle] * number_of_elements
        self.tau_n = [starting_throttle] * number_of_elements

        self.Tf = starting_time