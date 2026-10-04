import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from model import cartpole_dynamics
from pole_placement import K


# ------------------------------------------------------------
# Simulation settings
# ------------------------------------------------------------

t_start = 0.0
t_end = 8.0

t_eval = np.linspace(
    t_start,
    t_end,
    1500
)


# ------------------------------------------------------------
# Initial condition
# ------------------------------------------------------------

initial_state = np.array([
    0.0,                # Cart position [m]
    0.0,                # Cart velocity [m/s]
    np.deg2rad(5.0),    # Pendulum angle [rad]
    0.0                 # Pendulum angular velocity [rad/s]
])


# ------------------------------------------------------------
# Closed-loop nonlinear dynamics
# ------------------------------------------------------------

def closed_loop_dynamics(t, state):

    # State-feedback control law
    force = -(K @ state).item()

    return cartpole_dynamics(
        t,
        state,
        force=force
    )


# ------------------------------------------------------------
# Simulate
# ------------------------------------------------------------

solution = solve_ivp(
    closed_loop_dynamics,
    [t_start, t_end],
    initial_state,
    t_eval=t_eval,
    method="RK45",
    rtol=1e-8,
    atol=1e-10
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
# Reconstruct control force
# ------------------------------------------------------------

force_history = np.zeros_like(time)

for i in range(len(time)):

    state_i = solution.y[:, i]

    force_history[i] = -(K @ state_i).item()


# ------------------------------------------------------------
# Performance metrics
# ------------------------------------------------------------

# Maximum cart displacement
max_cart_displacement = np.max(np.abs(x))

# Maximum control effort
max_control_force = np.max(np.abs(force_history))

# Minimum pendulum angle
minimum_angle = np.min(theta_deg)


# ------------------------------------------------------------
# Settling-time function
# ------------------------------------------------------------

def calculate_settling_time(time, signal, tolerance):
    """
    Finds the first time after which the signal remains
    inside +/- tolerance.
    """

    inside_band = np.abs(signal) <= tolerance

    for i in range(len(time)):

        if np.all(inside_band[i:]):
            return time[i]

    return np.nan


# Pendulum considered settled inside +/- 0.1 degrees
angle_settling_time = calculate_settling_time(
    time,
    theta_deg,
    tolerance=0.1
)

# Cart considered settled inside +/- 1 mm
cart_settling_time = calculate_settling_time(
    time,
    x,
    tolerance=0.001
)


# ------------------------------------------------------------
# Print results
# ------------------------------------------------------------

print("\n--------------------------------")
print("POLE-PLACEMENT PERFORMANCE")
print("--------------------------------")

print(
    f"Maximum cart displacement: "
    f"{max_cart_displacement:.4f} m"
)

print(
    f"Maximum control force: "
    f"{max_control_force:.4f} N"
)

print(
    f"Minimum pendulum angle: "
    f"{minimum_angle:.4f} deg"
)

print(
    f"Pendulum settling time (+/-0.1 deg): "
    f"{angle_settling_time:.3f} s"
)

print(
    f"Cart settling time (+/-1 mm): "
    f"{cart_settling_time:.3f} s"
)

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
plt.title("Pole-Placement Control: Pendulum Angle")

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
plt.title("Pole-Placement Control: Cart Position")

plt.grid(True)
plt.tight_layout()

plt.show()


# ------------------------------------------------------------
# Plot control force
# ------------------------------------------------------------

plt.figure()

plt.plot(time, force_history)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time [s]")
plt.ylabel("Control force [N]")
plt.title("Pole-Placement Control Force")

plt.grid(True)
plt.tight_layout()

plt.show()