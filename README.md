# Cart-Pole Control Benchmark

Simulation and control of a nonlinear inverted-pendulum system using
pole-placement state feedback and Linear Quadratic Regulation (LQR).

## Project Overview

The cart-pole system is an inherently unstable nonlinear system.
The objective is to maintain the pendulum around the upright equilibrium
while simultaneously controlling cart motion using a horizontal force.

This project develops the system from its nonlinear equations of motion
through linearization, stability and controllability analysis, controller
design, nonlinear simulation, and quantitative controller comparison.

## Control Methods

- Nonlinear dynamic modeling
- State-space linearization
- Eigenvalue stability analysis
- Controllability analysis
- Pole-placement state feedback
- Linear Quadratic Regulator (LQR)
- Nonlinear closed-loop simulation
- Transient-performance analysis

## System States

The state vector is

x = [cart position, cart velocity, pendulum angle, angular velocity]

The control input is the horizontal cart force.

## Physical System

![Cart-Pole Model](figures/report/Figure_1_CartPole_Plant_Model.png)

## Open-Loop Behavior

The upright equilibrium is unstable.

Open-loop eigenvalues:

0, -0.1428, -5.6069, +5.5680

The positive eigenvalue confirms the instability observed in nonlinear simulation.

## Open-Loop Response

![Open Loop](figures/report/Figure_2_OpenLoop_Pendulum_Angle.png)

## Controllability

The controllability matrix has rank 4.

Therefore, the four-state system is fully controllable using the single
horizontal force input.

## Pole-Placement Controller

Desired poles:

-2.0, -2.5, -3.0, -3.5

State-feedback gain:

K = [-1.1774, -1.8885, -17.1829, -3.1354]

## LQR Controller

Weighting matrices:

Q = diag(10, 1, 100, 1)

R = 1

Resulting gain:

K = [-3.1623, -3.9764, -28.6082, -5.1827]

## Controller Comparison

| Metric                    | Pole Placement | LQR      |
| ------------------------- | --------------:| --------:|
| Maximum cart displacement | 0.1453 m       | 0.1019 m |
| Maximum control force     | 1.4995 N       | 2.4965 N |
| Minimum pendulum angle    | -2.127°        | -1.725°  |
| Pendulum settling time    | 3.501 s        | 3.266 s  |
| Cart settling time        | 4.446 s        | 4.835 s  |

The LQR controller provides improved pendulum regulation and reduced cart
travel, but requires significantly greater peak control effort.

## Controller Comparison

![Angle Comparison](figures/report/Figure_10_Pole_Place_vs_LQR_Pendulum_Angle.png)

![Cart Comparison](figures/report/Figure_11_Pole_Place_vs_LQR_Cart_Position.png)

![Force Comparison](figures/report/Figure_12_Pole_Place_vs_LQR_Control_Force.png)

## Software

- Python
- NumPy
- SciPy
- Matplotlib
- Python Control Systems Library
- Spyder

## Running the Project

Install dependencies:

pip install -r requirements.txt

Run the open-loop simulation:

python src/open_loop.py

Run the pole-placement simulation:

python src/simulate_pole_placement.py

Run the LQR simulation:

python src/simulate_lqr.py

Run the controller comparison:

python src/compare_controllers.py

## Future Work

- LQR Q/R tuning study
- Actuator saturation
- External disturbances
- Parameter uncertainty
- Monte Carlo robustness analysis
- Luenberger observer
- Kalman filter
- LQG control
