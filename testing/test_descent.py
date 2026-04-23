from lib.vectors import StateVector
from lib.descent import calculate_trajectory
from lib.obstacle import Ground

sv = StateVector(0, 30000, 0, 0)

tv = StateVector(200000, 80000, 0, 0)

calculate_trajectory(sv, tv, [Ground()], 100, 1000000, 45000, 1, 1)