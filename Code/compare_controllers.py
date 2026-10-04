import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from model import cartpole_dynamics
from pole_placement import K
from lqr_controller import K_lqr


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
# Settling-time function
# ------------------------------------------------------------

def calculate_settling_time(time, signal, tolerance):

    inside_band = np.abs(signal) <= tolerance

    for i in range(len(time)):
        if np.all(inside_band[i:]):
            return time[i]

    return np.nan


# ------------------------------------------------------------
# Pole-placement closed-loop dynamics
# ------------------------------------------------------------

def pole_placement_dynamics(t, state):

    force = -(K @ state).item()

    return cartpole_dynamics(
        t,
        state,
        force=force
    )


# ------------------------------------------------------------
# LQR closed-loop dynamics
# ------------------------------------------------------------

def lqr_dynamics(t, state):

    force = -(K_lqr @ state).item()

    return cartpole_dynamics(
        t,
        state,
        force=force
    )


# ------------------------------------------------------------
# Simulate pole placement
# ------------------------------------------------------------

solution_pp = solve_ivp(
    pole_placement_dynamics,
    [t_start, t_end],
    initial_state,
    t_eval=t_eval,
    method="RK45",
    rtol=1e-8,
    atol=1e-10
)


# ------------------------------------------------------------
# Simulate LQR
# ------------------------------------------------------------

solution_lqr = solve_ivp(
    lqr_dynamics,
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

time = solution_pp.t

x_pp = solution_pp.y[0]
theta_pp = np.rad2deg(solution_pp.y[2])

x_lqr = solution_lqr.y[0]
theta_lqr = np.rad2deg(solution_lqr.y[2])


# ------------------------------------------------------------
# Reconstruct control forces
# ------------------------------------------------------------

force_pp = np.zeros_like(time)
force_lqr = np.zeros_like(time)

for i in range(len(time)):

    force_pp[i] = -(K @ solution_pp.y[:, i]).item()
    force_lqr[i] = -(K_lqr @ solution_lqr.y[:, i]).item()


# ------------------------------------------------------------
# Performance metrics
# ------------------------------------------------------------

max_cart_pp = np.max(np.abs(x_pp))
max_cart_lqr = np.max(np.abs(x_lqr))

max_force_pp = np.max(np.abs(force_pp))
max_force_lqr = np.max(np.abs(force_lqr))

min_angle_pp = np.min(theta_pp)
min_angle_lqr = np.min(theta_lqr)

angle_settling_pp = calculate_settling_time(
    time,
    theta_pp,
    tolerance=0.1
)

angle_settling_lqr = calculate_settling_time(
    time,
    theta_lqr,
    tolerance=0.1
)

cart_settling_pp = calculate_settling_time(
    time,
    x_pp,
    tolerance=0.001
)

cart_settling_lqr = calculate_settling_time(
    time,
    x_lqr,
    tolerance=0.001
)


# ------------------------------------------------------------
# Print comparison table
# ------------------------------------------------------------

print("\n--------------------------------------------------")
print("CONTROLLER COMPARISON")
print("--------------------------------------------------")
print(f"{'Metric':35s} {'Pole Placement':>15s} {'LQR':>15s}")
print("-" * 70)
print(f"{'Maximum cart displacement [m]':35s} {max_cart_pp:15.4f} {max_cart_lqr:15.4f}")
print(f"{'Maximum control force [N]':35s} {max_force_pp:15.4f} {max_force_lqr:15.4f}")
print(f"{'Minimum pendulum angle [deg]':35s} {min_angle_pp:15.4f} {min_angle_lqr:15.4f}")
print(f"{'Pendulum settling time [s]':35s} {angle_settling_pp:15.3f} {angle_settling_lqr:15.3f}")
print(f"{'Cart settling time [s]':35s} {cart_settling_pp:15.3f} {cart_settling_lqr:15.3f}")


# ------------------------------------------------------------
# Comparison plot: Pendulum angle
# ------------------------------------------------------------

plt.figure()

plt.plot(
    time,
    theta_pp,
    label="Pole Placement"
)

plt.plot(
    time,
    theta_lqr,
    label="LQR"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time [s]")
plt.ylabel("Pendulum angle [deg]")
plt.title("Pendulum Angle: Pole Placement vs LQR")

plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


# ------------------------------------------------------------
# Comparison plot: Cart position
# ------------------------------------------------------------

plt.figure()

plt.plot(
    time,
    x_pp,
    label="Pole Placement"
)

plt.plot(
    time,
    x_lqr,
    label="LQR"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time [s]")
plt.ylabel("Cart position [m]")
plt.title("Cart Position: Pole Placement vs LQR")

plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


# ------------------------------------------------------------
# Comparison plot: Control force
# ------------------------------------------------------------

plt.figure()

plt.plot(
    time,
    force_pp,
    label="Pole Placement"
)

plt.plot(
    time,
    force_lqr,
    label="LQR"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time [s]")
plt.ylabel("Control force [N]")
plt.title("Control Force: Pole Placement vs LQR")

plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()