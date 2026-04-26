import math
from typing import Tuple
from matplotlib.patches import Patch, Rectangle, Ellipse
from .vectors import StateVector

class Obstacle:

    def get_penalty(self, state: StateVector) -> float:

        raise NotImplementedError("Inherited method not overridden.")
    
    def get_gradient(self, state: StateVector) -> Tuple[float, float]:

        raise NotImplementedError("Inherited method not overridden.")
    
    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:

        raise NotImplementedError("Inherited method not overridden.")

def acg(grad):

    def clip(x):

        return min(1e10, max(-1e10, x))
    
    return (clip(grad[0]), clip(grad[1]))

class Ground(Obstacle):

    def __init__(self):
        pass

    def get_penalty(self, state: StateVector):
        
        return math.pow(max(0, -state.y), 2)
    
    def get_gradient(self, state: StateVector):

        if state.y >= 0:
            return (0.0, 0.0)

        return acg((0.0, 2 * state.y))

    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:

        return Rectangle(
            (xmin, ymin), # bottom-left
            xmax - xmin, # width
            -ymin, # height up to y=0
            color='blue',
            alpha=0.3
        ) 

class RightBasicIntersect(Obstacle):

    def __init__(self, x_location, y_location):

        self.x = x_location
        self.y = y_location

    def get_penalty(self, state: StateVector):
        
        return math.pow(max(0, self.y - state.y), 2) * math.pow(max(0, self.x - state.x, 2))

    def get_gradient(self, state: StateVector):

        if state.x < self.x or state.y > self.y:

            return (0.0, 0.0)
        
        return acg((
            2 * math.pow(self.y - state.y, 2) * (state.x - self.x),
            -2 * math.pow(state.x - self.x, 2) * (self.y - state.y)
        ))
    
    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:

        return Rectangle(
            (self.x, ymin),
            xmax - self.x,
            self.y - ymin,
            color = 'magenta',
            alpha = .3
        )
    

class LeftBasicIntersect(Obstacle):

    def __init__(self, x_location, y_location):
        self.x = x_location
        self.y = y_location

    def get_penalty(self, state: StateVector):
        return (
            math.pow(max(0, self.y - state.y), 2)
            * math.pow(max(0, self.x - state.x), 2)
        )

    def get_gradient(self, state: StateVector):

        # Active region: state.x < self.x and state.y < self.y
        if state.x >= self.x or state.y >= self.y:
            return (0.0, 0.0)

        dx = self.x - state.x
        dy = self.y - state.y

        return acg((
            -2 * dx * math.pow(dy, 2),
            -2 * math.pow(dx, 2) * dy
        ))

    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:
        return Rectangle(
            (xmin, ymin),
            self.x - xmin,
            self.y - ymin,
            color='magenta',
            alpha=0.3
        )
    
class BothBasicIntersect(Obstacle):

    def __init__(self, x_left, x_right, y_location):
        self.xl = x_left
        self.xr = x_right
        self.y = y_location

    def get_penalty(self, state: StateVector):
        return (
            math.pow(max(0, state.x - self.xl), 2)
            * math.pow(max(0, self.xr - state.x), 2)
            * math.pow(max(0, self.y - state.y), 2)
        )

    def get_gradient(self, state: StateVector):

        # Active region: xl < x < xr AND y < Iy
        if state.x <= self.xl or state.x >= self.xr or state.y >= self.y:
            return (0.0, 0.0)

        dx_l = state.x - self.xl
        dx_r = self.xr - state.x
        dy = self.y - state.y

        grad_x = (
            2 * (dy ** 2) * dx_l * dx_r * (dx_r - dx_l)
        )

        grad_y = (
            -2 * (dx_l ** 2) * (dx_r ** 2) * dy
        )

        return acg((grad_x, grad_y))

    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:
        return Rectangle(
            (self.xl, ymin),
            self.xr - self.xl,
            self.y - ymin,
            color='magenta',
            alpha=0.3
        )
    
class RightComplexIntersect(Obstacle):

    def __init__(self, x_location, y_top, y_bottom):
        self.x = x_location
        self.y_top = y_top
        self.y_bottom = y_bottom

    def get_penalty(self, state: StateVector):
        return (
            math.pow(max(0, state.x - self.x), 2)
            * math.pow(max(0, self.y_top - state.y), 2)
            * math.pow(max(0, state.y - self.y_bottom), 2)
        )

    def get_gradient(self, state: StateVector):
        if state.x <= self.x or state.y >= self.y_top or state.y <= self.y_bottom:
            return (0.0, 0.0)

        dx = state.x - self.x
        dy_top = self.y_top - state.y
        dy_bottom = state.y - self.y_bottom

        grad_x = 2 * dx * dy_top**2 * dy_bottom**2
        grad_y = 2 * dx**2 * dy_top * dy_bottom * (dy_top - dy_bottom)

        return acg((grad_x, grad_y))

    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:
        return Rectangle(
            (self.x, self.y_bottom),
            xmax - self.x,
            self.y_top - self.y_bottom,
            color='magenta',
            alpha=0.3
        )


class LeftComplexIntersect(Obstacle):

    def __init__(self, x_location, y_top, y_bottom):
        self.x = x_location
        self.y_top = y_top
        self.y_bottom = y_bottom

    def get_penalty(self, state: StateVector):
        return (
            math.pow(max(0, self.x - state.x), 2)
            * math.pow(max(0, self.y_top - state.y), 2)
            * math.pow(max(0, state.y - self.y_bottom), 2)
        )

    def get_gradient(self, state: StateVector):
        if state.x >= self.x or state.y >= self.y_top or state.y <= self.y_bottom:
            return (0.0, 0.0)

        dx = self.x - state.x
        dy_top = self.y_top - state.y
        dy_bottom = state.y - self.y_bottom

        grad_x = -2 * dx * dy_top**2 * dy_bottom**2
        grad_y = 2 * dx**2 * dy_top * dy_bottom * (dy_top - dy_bottom)

        return acg((grad_x, grad_y))

    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:
        return Rectangle(
            (xmin, self.y_bottom),
            self.x - xmin,
            self.y_top - self.y_bottom,
            color='magenta',
            alpha=0.3
        )


class BothComplexIntersect(Obstacle):

    def __init__(self, x_left, x_right, y_top, y_bottom):
        self.xl = x_left
        self.xr = x_right
        self.y_top = y_top
        self.y_bottom = y_bottom

    def get_penalty(self, state: StateVector):
        return (
            math.pow(max(0, state.x - self.xl), 2)
            * math.pow(max(0, self.xr - state.x), 2)
            * math.pow(max(0, self.y_top - state.y), 2)
            * math.pow(max(0, state.y - self.y_bottom), 2)
        )

    def get_gradient(self, state: StateVector):
        if (
            state.x <= self.xl
            or state.x >= self.xr
            or state.y >= self.y_top
            or state.y <= self.y_bottom
        ):
            return (0.0, 0.0)

        dx_l = state.x - self.xl
        dx_r = self.xr - state.x
        dy_top = self.y_top - state.y
        dy_bottom = state.y - self.y_bottom

        grad_x = (
            2
            * dx_l
            * dx_r
            * (dx_r - dx_l)
            * dy_top**2
            * dy_bottom**2
        )

        grad_y = (
            2
            * dx_l**2
            * dx_r**2
            * dy_top
            * dy_bottom
            * (dy_top - dy_bottom)
        )

        return acg((grad_x, grad_y))

    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:
        return Rectangle(
            (self.xl, self.y_bottom),
            self.xr - self.xl,
            self.y_top - self.y_bottom,
            color='magenta',
            alpha=0.3
        )
    
class EllipseIntersect(Obstacle):

    def __init__(self, x_center, y_center, a, b):
        self.ex = x_center
        self.ey = y_center
        self.a = a
        self.b = b

    def get_penalty(self, state: StateVector):
        dx = state.x - self.ex
        dy = state.y - self.ey

        q = (dx * dx) / (self.a * self.a) + (dy * dy) / (self.b * self.b)

        # avoid singularity at ellipse center
        if q <= 1e-12:
            q = 1e-12

        return math.pow(max(0, (1 / q) - 1), 2)

    def get_gradient(self, state: StateVector):
        dx = state.x - self.ex
        dy = state.y - self.ey

        q = (dx * dx) / (self.a * self.a) + (dy * dy) / (self.b * self.b)

        # outside ellipse -> zero penalty / zero gradient
        if q >= 1.0:
            return (0.0, 0.0)

        # avoid singularity at center
        if q <= 1e-12:
            q = 1e-12

        h = (1 / q) - 1

        dq_dx = 2 * dx / (self.a * self.a)
        dq_dy = 2 * dy / (self.b * self.b)

        # P = (1/q - 1)^2
        # dP/dx = -2(1/q - 1)(1/q^2)(dq/dx)
        grad_x = -2 * h * dq_dx / (q * q)
        grad_y = -2 * h * dq_dy / (q * q)


        return acg((1e13 * grad_x, 1e13 * grad_y))

    def get_patch(self, xmin, xmax, ymin, ymax) -> Patch:
        return Ellipse(
            (self.ex, self.ey),
            2 * self.a,
            2 * self.b,
            color='magenta',
            alpha=0.3
        )