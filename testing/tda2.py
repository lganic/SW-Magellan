import math
from lib.vectors import StateVector
from lib.descent_cg import calculate_trajectory, fletcher_reeves, polak_ribiere, hestenes_stiefel
from lib.descent_steepest import calculate_trajectory as calculate_trajectory_steepest
from lib.obstacle import Ground, RightBasicIntersect, LeftBasicIntersect, BothBasicIntersect, LeftComplexIntersect, RightComplexIntersect, BothComplexIntersect, EllipseIntersect

def err_dist(p1: StateVector, p2: StateVector):
    return math.sqrt(math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2))

def err_vel(p1: StateVector, p2: StateVector):
    return math.sqrt(math.pow(p1.vx - p2.vx, 2) + math.pow(p1.vy - p2.vy, 2))

sv = StateVector(0, 0, 0, 500)

target_moon = StateVector(200000, 80000, 0, 0)
target_orbit = StateVector(20000, 300000, 0, 0)

obstacles = [
    Ground(),
    LeftBasicIntersect(-128000, 40000),
    LeftComplexIntersect(-40000, 128000, 40000),
    BothComplexIntersect(40000, 128000, 128000, 40000),
    BothBasicIntersect(128000, 184000, 128000),
    RightBasicIntersect(216000, 128000),
    EllipseIntersect(100000,  128000, 60000, 60000)
]

states_steepest, params_steepest, iteration_steepest, all_objective_losses_steepest = calculate_trajectory_steepest(sv, target_moon, obstacles, 500, 200000, 4500, 1 , 1)
states_fletcher, params_fletcher, iteration_fletcher, all_objective_losses_fletcher = calculate_trajectory(sv, target_moon, obstacles, 500, 200000, 4500, 1 , 1, fletcher_reeves)
states_polak, params_polak, iteration_polak, all_objective_losses_polak = calculate_trajectory(sv, target_moon, obstacles, 500, 200000, 4500, 1 , 1, polak_ribiere)
states_hestenes, params_hestenes, iteration_hestenes, all_objective_losses_hestenes = calculate_trajectory(sv, target_moon, obstacles, 500, 200000, 4500, 1 , 1, hestenes_stiefel)

loss = all_objective_losses_steepest[-1]
distance = err_dist(target_moon, states_steepest[-1])
velocity = err_vel(target_moon, states_steepest[-1])
time = params_steepest.Tf

print("Steepest: ", distance, velocity, time, loss)