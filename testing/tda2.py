from lib.vectors import StateVector
from lib.descent_live import calculate_trajectory
from lib.obstacle import Ground

sv = StateVector(0, 0, 0, 0)

tv = StateVector(10000, 30000, 0, 0)

calculate_trajectory(sv, tv, [Ground()], 100, 200000, 4500, 1, 1)