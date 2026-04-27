def golden_search(function, low, high, tolerance=1e-6):
    gr = (1 + 5 ** 0.5) / 2

    x1 = high - (high - low) / gr
    x2 = low + (high - low) / gr

    f1 = function(x1)
    f2 = function(x2)

    while abs(high - low) > tolerance:
        if f1 > f2:
            low = x1
            x1 = x2
            f1 = f2
            x2 = low + (high - low) / gr
            f2 = function(x2)
        else:
            high = x2
            x2 = x1
            f2 = f1
            x1 = high - (high - low) / gr
            f1 = function(x1)

    return (low + high) / 2

if __name__ == '__main__':

    def func(x):

        return 3 * x * x + 4 * x + 5
    
    print(golden_search(func, -5, 5))