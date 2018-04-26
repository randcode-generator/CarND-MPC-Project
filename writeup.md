# CarND-Controls-MPC
This is a writeup for the Model Predictive Control (MPC) project. We will use MPC to control the steering wheel and acceleration.

# Background
In this project, we are given the waypoints to the track. We will use MPC to predict the trajectory by selecting a trajectory with the minimal cost. This will allow us to obtain the best steering angle and acceleration at that given time.

# The Model
We use IPOPT (Interior Point OPTimizer) to find the best steering angle and acceleration based on the state variables and coefficients. 

For the state, it requires the following:

| state     |                                   |
| --------- | --------------------------------- |
| x         | x position of the car             |
| y         | y position of the car             |
| psi       | angle of the car from x axis      |
| velocity  | speed of the car                  |
| CTE       | error of car from waypoint path   |
| psi error | error of angle of car from x axis |

For coefficients, it is calculate by using `polyfit` for waypoints of the track.

The following costs were used:
```
fg[0] += 4000 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
fg[0] += 4000 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
```
Heavy emphasis on the CTE and the psi error. The car going max speed was not heavily emphasized.

The following constrains were used:
```
for (size_t i = 0; i < delta_start; i++) {
  vars_lowerbound[i] = -1.0e19;
  vars_upperbound[i] = 1.0e19;
}

for (size_t i = delta_start; i < a_start; i++) {
  vars_lowerbound[i] = -0.436332*Lf;
  vars_upperbound[i] = 0.436332*Lf;
}

for (size_t i = a_start; i < n_vars; i++) {
  vars_lowerbound[i] = -1.0;
  vars_upperbound[i] = 1.0;
}
```
The bounds for x, y, psi, velocity, CTE, and psi error is between -1.0e19 and 1.0e19. 

The bounds for delta (steering angle) is between negative radian 25 degrees times Lf and positive radian 25 degrees times Lf

The bounds for acceleration is between negative 1 and positive 1.

# Timestep Length and Elapsed Duration
I chose N=10 and dt=0.1 because the car was able to achieve max speeds of 91 mph and did not touch the sides while turning. It also kept on the waypoints.

| N | dt  | observation |
| - | --- | ------ |
| 8 | 0.1 | slow around turns and only achieved high speeds of 70 mph |
| 10 | 0.2 | slow around turns and deviates from waypoint sometimes |
| 8 | 0.2 | slow around turns and deviates from waypoint sometimes |

# Polynomial Fitting and MPC Preprocessing
I use the kinematic model to predict the position of the car 0.1 seconds later to accommodate for the 100ms delay.

I converted the waypoints from map coordinates to car coordinates.

# Model Predictive Control with Latency
Answered in the section above