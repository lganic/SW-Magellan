import math

def golden_search(function, min_range, max_range, tolerance = 1e-8):

    if max_range - min_range < tolerance:

        return (max_range + min_range) / 2

    golden_ratio = (1 + math.sqrt(5)) / 2

    x1 = max_range - (max_range - min_range) / golden_ratio
    x2 = min_range + (max_range - min_range) / golden_ratio

    fx1 = function(x1)
    fx2 = function(x2)

    if fx1 < fx2:

        return golden_search(function, min_range, x2, tolerance)
    
    return golden_search(function, x1, max_range, tolerance)

if __name__ == '__main__':

    def func(x):

        return 3 * x * x + 4 * x + 5
    
    print(golden_search(func, -5, 5))