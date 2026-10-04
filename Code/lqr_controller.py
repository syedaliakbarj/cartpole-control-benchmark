import numpy as np
import control

from linear_model import A, B


# ------------------------------------------------------------
# LQR weighting matrices
# ------------------------------------------------------------

Q = np.diag([
    10.0,    # Cart position
    1.0,     # Cart velocity
    100.0,   # Pendulum angle
    1.0      # Pendulum angular velocity
])

R = np.array([
    [1.0]
])


# ------------------------------------------------------------
# Compute LQR controller
# ------------------------------------------------------------

K_lqr, S, E = control.lqr(
    A,
    B,
    Q,
    R
)


# ------------------------------------------------------------
# Print results
# ------------------------------------------------------------

if __name__ == "__main__":

    print("LQR gain K:")
    print(K_lqr)

    print("\nClosed-loop eigenvalues:")
    print(E)