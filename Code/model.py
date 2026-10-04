import numpy as np
"""
Cart-Pole Control Project
File: model.py

Purpose:
Define the physical parameters of the cart-pole system.
"""

# Physical parameters

M = 0.5      # Cart mass [kg]
m = 0.2      # Pendulum mass [kg]

L = 0.6      # Full pendulum length [m]
l = L / 2    # Distance from pivot to centre of mass [m]

b = 0.1      # Cart friction coefficient [N s/m]
g = 9.81     # Gravitational acceleration [m/s^2]

I = m * L**2 / 12   # Pendulum moment of inertia [kg m^2]


def cartpole_dynamics(t, state, force=0.0):

    x, x_dot, theta, theta_dot = state

    # Mass matrix
    mass_matrix = np.array([
        [M + m,                  m * l * np.cos(theta)],
        [m * l * np.cos(theta), I + m * l**2]
    ])

    # Right-hand side of equations of motion
    rhs = np.array([
        force
        - b * x_dot
        + m * l * theta_dot**2 * np.sin(theta),

        m * g * l * np.sin(theta)
    ])

    # Solve for accelerations
    x_ddot, theta_ddot = np.linalg.solve(
        mass_matrix,
        rhs
    )

    # Return derivative of state
    return np.array([
        x_dot,
        x_ddot,
        theta_dot,
        theta_ddot
    ])
    
if __name__ == "__main__":

    print("Cart mass M =", M, "kg")
    print("Pendulum mass m =", m, "kg")
    print("Pendulum length L =", L, "m")
    print("Centre of mass distance l =", l, "m")
    print("Pendulum inertia I =", I, "kg m^2")

    # Test the nonlinear model
    test_state = [
        0.0,                # Cart position [m]
        0.0,                # Cart velocity [m/s]
        np.deg2rad(5.0),    # Pendulum angle [rad]
        0.0                 # Pendulum angular velocity [rad/s]
    ]

    result = cartpole_dynamics(
        t=0.0,
        state=test_state,
        force=0.0
    )

    print("\nState derivative:")
    print(result)