# Disturbance Observer and Loop-shaped Controller

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=simorxb/loopshaping-and-disturbance-observer)

## Summary
This project implements a disturbance observer and a loop-shaped controller to manage the behavior of a control system subjected to disturbances. The project includes detailed models, transfer functions, and control architecture descriptions, focusing on achieving zero steady-state error in response to a step input, even with constant disturbances.

## Project Overview
The primary objective of this project is to design and simulate a control system that leverages an extended state observer (ESO) alongside a loop-shaped controller. The system controls an object of mass $m = 10$ kg that slides on a surface with damping, using the force $F$ as the control input, while compensating for an unknown disturbance $F_d$.

### Plant
The dynamic equation of the plant is given as:

$$
m \ddot{z} = F - k \dot{z} + F_d
$$

where:
- $m = 10$ kg (mass of the object)
- $k = 0.5$ Ns/m (damping coefficient)
- $F$ is the control input force
- $F_d$ is the unknown disturbance acting on the control input

### Control Architecture
The control system is designed with the following components:
1. **Loop-shaped Controller**: Ensures a desired closed-loop response to step inputs with good robustness.
2. **Extended State Observer (ESO)**: Estimates the system states and disturbance $F_d$.
3. **Compensation Mechanism**: Subtracts the estimated disturbance $F_d$ from the control input generated by the compensator.

### Loop-shaped Controller Design
The controller is designed using a loop-shaping technique targeting a specific closed-loop response with transfer function:

$$
T_d = \frac{1}{(\tau s + 1)^2}
$$

where $\tau = 0.2$.

Using MATLAB’s `loopsyn` command with $\alpha = 0.5$, the controller transfer function is obtained as:

$$
K(s) = \frac{301s^2 + 4206s + 2091}{s^2 + 24.56s + 180.7}
$$

### Extended State Observer Design
The extended state observer includes the disturbance $F_d$. The extended state vector is defined as:

$$
x = [\dot{z} \, z \, F_d]
$$

The corresponding state-space matrices are:

$$
A_e = 
\begin{bmatrix}
-\frac{k}{m} & 0 & \frac{1}{m} \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
, \quad
B_e = 
\begin{bmatrix}
\frac{1}{m} \\
0 \\
0
\end{bmatrix}
, \quad
C_e = 
\begin{bmatrix}
0 & 1 & 0
\end{bmatrix}
, \quad
D_e = 0
$$

The observer poles are placed farther left from the closed-loop transfer function poles to ensure faster disturbance estimation:

$$
L = \text{place}(A_e^T, C_e^T, [ -3, -3.5, -4 ])^T
$$

### Control System Discretization
The system is discretized using a sample time $T_s = 0.1$. The continuous-time controller and observer are converted to discrete-time using MATLAB’s `c2d` function:

```matlab
K_d = c2d(K, T_s, 'matched')
obs_d = c2d(ss(A_e, B_e, C_e, D_e), T_s)
```

### Simulation
The simulation includes the following settings to make the scenario more realistic:

White noise is added to the measured controlled variable.
- The disturbance $F_d$ steps to 20 N after 10 seconds.
- The control input is saturated at ± 50N.

### Simulation Results
The results show that the extended state observer effectively estimates the disturbance $F_d$ and compensates for it, ensuring zero steady-state error in response to a step input.

### Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

### Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
