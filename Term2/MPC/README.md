# Model Predictive Controller (*MPC*)

## The Model
**Kinematic Model includes:**
1. the vehicle's x and y coordinates
2. orientation angle (psi)
3. velocity.
4. cross-track error
5. psi error (epsi). 

**Actuator outputs are:**
1. acceleration.
2. delta (steering angle). 

The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on these equations:

```C++
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Timestep Length and Elapsed Duration (N & dt)
N = 15 and dt = 0.1

Which means the optimizer is considering 1s duration in which to determine a corrective trajectory. Changing N/dt often result in unexpected behavior. Other values tried includeing 10 / 0.1 and others.

## Polynomial Fitting and MPC Preprocessing
The waypoints are preprocessed by transforming them to the vehicle's perspective. This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also 0. 

## Model Predictive Control with Latency
The approach to deal with latency was to add additional cost funciton penalizing the combination of velocity and delta which results in much more controlled behavior.
