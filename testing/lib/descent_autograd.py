import math
from typing import List
from .vectors import StateVector, ParameterVector
from .sim import sim
from .obstacle import Obstacle
from .gradient import get_gradients
from .objective_function import objective_function
from matplotlib import pyplot as plt
import keyboard
import autograd.numpy as np
from autograd import grad
from scipy.optimize import minimize

from .vectors import StateVector, ParameterVector
from .sim import sim
from .objective_function import make_autograd_objective, pack_params, unpack_params

def calculate_trajectory(
    starting_state: StateVector,
    end_state: StateVector,
    obstacles,
    N: int,
    engine_thrust: float,
    starting_mass: float,
    fuel_consumption_rate: float,
    fuel_density: float,
):
    params0 = ParameterVector(N)

    constants = {
        "Lambda B": 4e-9,
        "Lambda Mu": 1e-3,
        "Lambda P": 1e-2,
        "Lambda V": 3e2,
    }

    f = make_autograd_objective(
        starting_state=starting_state,
        target_state=end_state,
        obstacles=obstacles,
        N=N,
        engine_thrust=engine_thrust,
        starting_mass=starting_mass,
        fuel_consumption_rate=fuel_consumption_rate,
        fuel_density=fuel_density,
        tuning_constants=constants,
    )

    df = grad(f)

    x0 = pack_params(params0)

    bounds = [(None, None)] * N + [(0.0, 1.0)] * N + [(None, None)]

    history = []

    def callback(xk):
        loss = f(xk)
        history.append(loss)
        print("Objective:", loss)

    result = minimize(
        fun=f,
        x0=x0,
        jac=df,
        method="L-BFGS-B",   # better than CG if tau has bounds
        bounds=bounds,
        callback=callback,
        options={
            "maxiter": 1000,
            "gtol": 1e-6,
            "ftol": 1e-12,
            "maxls": 50,
        },
    )

    final_params = unpack_params(result.x, N)

    print("Optimization success:", result.success)
    print("Message:", result.message)
    print("Final objective:", result.fun)
    print("Iterations:", result.nit)

    return final_params, history, result