from lib.vectors import StateVector
from lib.descent_cg import calculate_trajectory, fletcher_reeves, polak_ribiere, hestenes_stiefel
from lib.obstacle import Ground, RightBasicIntersect, LeftBasicIntersect, BothBasicIntersect, LeftComplexIntersect, RightComplexIntersect, BothComplexIntersect, EllipseIntersect

sv = StateVector(0, 0, 0, 500)

tv = StateVector(200000, 80000, 0, 0)

obstacles = [
    Ground(),
    LeftBasicIntersect(-128000, 40000),
    LeftComplexIntersect(-40000, 128000, 40000),
    BothComplexIntersect(40000, 128000, 128000, 40000),
    BothBasicIntersect(128000, 184000, 128000),
    RightBasicIntersect(216000, 128000),
    EllipseIntersect(100000,  128000, 60000, 60000)
]

calculate_trajectory(sv, tv, obstacles, 500, 200000, 4500, 0 , 1, polak_ribiere)