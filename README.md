# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Overview
This project involves developing code for Model Predictive Control to drive the simulated car around a vehicle track. Waypoints from the track are used to navigate the car. Steps to complete this project are:
* Fit a line based on the waypoints.
* Implement Model Predictive Control calculation.
* Calculate Steering and throttle values based on the MPC calculation.

## Implementation

#### Model
The model used is a kinematic model which does not consider the tires and road. The model equation are as follows,

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```
* Car position is given by 'x, y'
* Car heading is givne by 'psi'
* Car velocity is gievn by 'v'
* Cross track error is given by 'cte'
* Orientation error is given by 'epsi'
* Throttle is given by 'a'
* Steering angle is given by 'delta'
* Distance between center of car mass to car front is given by 'Lf'

Values of throttle and steering angle are calculated such that the car drives along the track without leaving the outer boundaries and also try to reach the maximum throttle set.

### N and dt
* N is the number of timesteps the model predicts in ahead.
* dt is the length of each timestep.

#### Tuning
Initially I started of with N=10 and dt=0.1. The car did drive pretty smoothly around the track. But, the green line used to not follow the path at the tip. To avoid this I though increasing the value of N would help. The value of N was increased to 15 while keeping the dt value to 0.1, the car did drive smoothly for half the track, but at one of the turning, the car went off the track. The value of N was reduced to 12, and it was observed that the car completed the track with a smooth drive. So the value of N was kept at 12. Now the value of dt was changed to 0.2 and it was observed that the car used to brake often and the drive was no more smooth. So I changed the value to 0.05 and I could observe the car to wobble in the track and was not going straight. Hence the value of N was kept at 12 and the value of dt to 0.1.

### Latency
The thread in main.cpp runs at an interval of 100ms. If this latency is not considered in the implementation, then the values that the car would get would be for the previous 100ms and the calculated throttle and steering values would be for the previous time and not for current time. This would lead to a wrong behaviour of car. Hence the kinematic model code was tuned to predict 100ms in advance by multiplying 'delay' parameter which was set to 0.1 (100 ms). This can be observed in main.cpp.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
