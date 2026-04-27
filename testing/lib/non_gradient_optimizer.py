'''
This file will implement a non gradient optimization technique. 

I will implement this as a single dimension Nelder Mead
'''

# So I can be lazy:
class FuncCache:
    def __init__(self, function):

        self.func = function

        self.table = {}

    def __call__(self, value):

        if value in self.table:
            return self.table[value]
        
        result = self.func(value)

        self.table[value] = result

        return result

class PunishOOO:
    def __init__(self, function, low, high):

        self.func = function
        self.low = low
        self.high = high

    def __call__(self, value):

        if value < self.low or value > self.high:

            return float('inf')
        
        return self.func(value)

def nelder_mead_1d(func_to_optimize, x1, x2, tolerance = 1e-3):

    func_noclamp = FuncCache(func_to_optimize) # Wrap this, so I can be lazy with my notation, and have this still be an optimized method
    func = PunishOOO(func_noclamp, x1, x2) # Force to be in range.

    alpha = 1 # Reflection
    gamma = 2 # Expansion
    p = .5 # Contraction
    sigma = .5 # Shrink

    while True:

        # Order the points

        if func(x1) < func(x2):
            best = x1
            worst = x2
        else:
            best = x2
            worst = x1

        # Check convergence
        if abs(worst - best) < tolerance:
            return best
        
        # Find the centroid. We are in 1D so thats just the best
        centroid = best

        # Reflection
        xr = centroid + alpha * (centroid - worst)

        if func(xr) < func(best):
            # Expansion

            xe = centroid + gamma * (xr - centroid)

            if func(xe) < func(xr):
                worst = xe
            else:
                worst = xr

        elif func(xr) < func(worst):

            worst = xr

        else:

            # Contraction

            xc = centroid + p * (worst - centroid)

            if func(xc) < func(worst):
                worst = xc

            else:
                # Shrink
                worst = best + sigma * (worst - best)

        # Update points
        if func(x1) < func(x2):
            x1 = best
            x2 = worst
        else:
            x2 = best
            x1 = worst


if __name__ == '__main__':

    def func(x):

        return 3 * x * x + 4 * x + 5
    
    print(nelder_mead_1d(func, -5, 5))