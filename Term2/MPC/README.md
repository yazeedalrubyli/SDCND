# Model Predictive Controller (MPC)

## The Model
**Kinematic Model:**
1. Vehicle's x and y coordinates.
2. Orientation Angle (psi).
3. Velocity (v).
4. Cross-track Error (cte).
5. Orientation Error (epsi). 

**Actuators:**
1. Acceleration (a).
2. Steering Angle (delta). 

The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on these equations:

```C++
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
x,y denote the position of the vehicle, psi the orientation angle/heading direction, v is the velocity cte the cross-track error and epsi the orientation error. Lf is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability.

State ~> [x,y,psi,v]

Control Input (Actuators) ~> [delta,a]

## Timestep Length and Elapsed Duration (N & dt)
`N = 10` and `dt = 0.1`. These values were chosen for which the car drive smoothly around the track for velocities range from 0 to around 55mph. The prediction horizon `T` is the duration over which future predictions are made which is equal to `N x dt`, Where `N` is the number of timesteps in the horizon and `dt` is how much time elapses between actuation, in our case `N = 10` and `dt = 0.1`, then T would be 1 second. Choosing dt to be smaller is better because it makes it easy to accurately approximate a continuous reference trajectory. In addition, choosing `N` to be large isn't always better because it will consider further trajectory's points in the model which may produce inconsistent behavior.

## Polynomial Fitting and MPC Preprocessing
The waypoints are preprocessed by transforming them to the vehicle's perspective. This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also 0. 

## Model Predictive Control with Latency
Unlike PID controller, MPC can model the latency easily. A contributing factor to latency is actuator dynamics. For example, the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.
Thus, MPC can deal with latency much more effectively.
