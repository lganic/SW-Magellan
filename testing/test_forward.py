import matplotlib.pyplot as plt
from lib.sim import sim
from lib.vectors import StateVector, ParameterVector

N = 100

param = ParameterVector(N, starting_time = 30)

initial_state = StateVector(0, 0, 0, 0)

Thrust = 500000
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
vx = [s.vx for s in states]
vy = [s.vy for s in states]

# Plot trajectory
plt.figure(figsize=(8, 6))
plt.plot(x, y, label="Trajectory", linewidth=2)

# Quiver 
plt.quiver(x, y, vx, vy, angles='xy', scale_units='xy', scale=1)

# Labels / formatting
plt.xlabel("x")
plt.ylabel("y")
plt.title("Trajectory")
plt.axis('equal')
plt.grid(True)
plt.legend()

plt.show()