import numpy as np
import control

from linear_model import A, B

# ------------------------------------------------------------
# Desired closed-loop poles
# ------------------------------------------------------------

desired_poles = [
    -2.0,
    -2.5,
    -3.0,
    -3.5
]

# ------------------------------------------------------------
# State-feedback gain
# ------------------------------------------------------------

K = control.place(A, B, desired_poles)

# ------------------------------------------------------------
# Closed-loop dynamics
# ------------------------------------------------------------

A_closed_loop = A - B @ K

closed_loop_eigenvalues = np.linalg.eigvals(
    A_closed_loop
)

# ------------------------------------------------------------
# Print results only when run directly
# ------------------------------------------------------------

if __name__ == "__main__":

    print("State-feedback gain K:")
    print(K)

    print("\nClosed-loop eigenvalues:")
    print(closed_loop_eigenvalues)

