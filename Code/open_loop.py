import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from model import cartpole_dynamics


# ------------------------------------------------------------
# Simulation time
# ------------------------------------------------------------

t_start = 0.0
t_end = 5.0

t_eval = np.linspace(t_start, t_end, 1000)


# ------------------------------------------------------------
# Initial condition
# ------------------------------------------------------------

initial_state = [
    0.0,                # Cart position [m]
    0.0,                # Cart velocity [m/s]
    np.deg2rad(5.0),    # Pendulum angle [rad]
    0.0                 # Pendulum angular velocity [rad/s]
]


# ------------------------------------------------------------
# Open-loop system: no control force
# ------------------------------------------------------------

def open_loop_dynamics(t, state):
    return cartpole_dynamics(
        t,
        state,
        force=0.0
    )


# ------------------------------------------------------------
# Solve differential equations
# ------------------------------------------------------------

solution = solve_ivp(
    open_loop_dynamics,
    [t_start, t_end],
    initial_state,
    t_eval=t_eval,
    method="RK45"
)


# ------------------------------------------------------------
# Extract states
# ------------------------------------------------------------

time = solution.t

x = solution.y[0]
x_dot = solution.y[1]
theta = solution.y[2]
theta_dot = solution.y[3]

theta_deg = np.rad2deg(theta)


# ------------------------------------------------------------
# Plot pendulum angle
# ------------------------------------------------------------

plt.figure()

plt.plot(time, theta_deg)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time [s]")
plt.ylabel("Pendulum angle [deg]")
plt.title("Open-Loop Pendulum Angle")

plt.grid(True)
plt.tight_layout()

plt.show()

# ------------------------------------------------------------
# Plot cart position
# ------------------------------------------------------------

plt.figure()

plt.plot(time, x)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time [s]")
plt.ylabel("Cart position [m]")
plt.title("Open-Loop Cart Position")

plt.grid(True)
plt.tight_layout()

plt.show()