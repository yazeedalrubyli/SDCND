# PID Controller
## P, I, D Effect
**P** (Proportional) it causes the car to steer proportional to the lane center. If the car is far to the right it steers hard to the left, if it's slightly to the left it steers slightly to the right.

**D** (Differential) it aims at flattening the error trajectory into a horizontal line. A properly tuned D parameter will cause the car to approach the center line smoothly without overshoot.

**I** (Integral) if applied force is not enough to bring the error to zero, this force will be increased as time passes. The bias can take several forms, such as a steering drift (as in the Control unit lessons), but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.

## Hyperparameters
Hyperparameters were chosen manually after couple attempts and the car was smoothly moving around the track. We can extend the project further by implementing the Twiddle algorithm to automate parameter optimization.
