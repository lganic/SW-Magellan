import matplotlib.pyplot as plt
import math
from lib.sim import sim
from lib.vectors import StateVector, ParameterVector

N = 100

param = ParameterVector(N, starting_time = 60, starting_throttle=1, starting_angle=2.6)

initial_state = StateVector(0, 0, 500, 0)

Thrust = 1000000
Mass = 45000

Fuel_Consumption_Rate = 1
Fuel_Density = 1

states = sim(
    initial_state,
    param,
    Thrust,
    Mass,
    Fuel_Consumption_Rate,
    Fuel_Density
)

# Extract data
x = [s.x for s in states]
y = [s.y for s in states]
arrow_x = [thrust * math.cos(angle) for angle, thrust in zip(param.theta_n, param.tau_n)] + [0]
arrow_y = [thrust * math.sin(angle) for angle, thrust in zip(param.theta_n, param.tau_n)] + [0]

# Plot trajectory
plt.figure(figsize=(8, 6))
plt.plot(x, y, label="Trajectory", linewidth=2)

# Quiver 
plt.quiver(x, y, arrow_x, arrow_y, angles='xy', scale_units='xy', scale=2e-3)

# Labels / formatting
plt.xlabel("x")
plt.ylabel("y")
plt.title("Trajectory")
plt.axis('equal')
plt.grid(True)
plt.legend()

plt.show()