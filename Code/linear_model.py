import numpy as np
import control
from model import M, m, l, b, g, I


# ------------------------------------------------------------
# Intermediate quantities
# ------------------------------------------------------------

J = I + m * l**2

delta = (M + m) * J - (m * l)**2


# ------------------------------------------------------------
# State-space matrices
# ------------------------------------------------------------

A = np.array([
    [0.0, 1.0, 0.0, 0.0],

    [0.0,
     -(J * b) / delta,
     -(m**2 * g * l**2) / delta,
     0.0],

    [0.0, 0.0, 0.0, 1.0],

    [0.0,
     (m * l * b) / delta,
     ((M + m) * m * g * l) / delta,
     0.0]
])


B = np.array([
    [0.0],
    [J / delta],
    [0.0],
    [-(m * l) / delta]
])

# ------------------------------------------------------------
# Open-loop eigenvalues
# ------------------------------------------------------------

eigenvalues = np.linalg.eigvals(A)


# ------------------------------------------------------------
# Controllability analysis
# ------------------------------------------------------------

controllability_matrix = control.ctrb(A, B)

rank_controllability = np.linalg.matrix_rank(
    controllability_matrix
)

# ------------------------------------------------------------
# Print everything
# ------------------------------------------------------------


if __name__ == "__main__":

    print("A matrix:")
    print(A)

    print("\nB matrix:")
    print(B)

    print("\nOpen-loop eigenvalues:")
    print(eigenvalues)

    print("\nControllability matrix:")
    print(controllability_matrix)

    print("\nRank of controllability matrix:")
    print(rank_controllability)