# Model Predictive Control project
Self-Driving Car Engineer Nanodegree Program

---

## Overview

In this project I'he utilized Model Predictive Controller (MPC) to control a car moving on the track. Controller receives track center points as well as coordinates, angle in local coordinate system and velocity. The output of controller is steering angle and throttle signals. Also there are artificial  100 ms delay in controller to make things more interesting. Video of result can be found [here](https://youtu.be/JdIFCMO3Eeo).

## Compile

This project require [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) to be installed. This repository includes two files that can be used to set up and install it for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make

Dependencies:

* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4
* Ipopt
* CppAD

## Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)


To run the MPC algorithm use

```

./build/mpc
```

command in project directory.


## Reflection

Here I'll write some thoughts about MPC behavior and parameters tuning.

### Model description

The car state vector has 4 parameters `[x, y, psi, v]` - two coordinates of the car and angle in local coordinate system, signed speed of the car. Also there are two actuators `[delta, a]` - steering angle and acceleration.

The movement model of the car can be written with next equations:

```
x(t+1) = x(t)+ v(t)*cos(psi(t))*dt
y(t+1) = y(t)+ v(t)*sin(psi(t))*dt
psi(t+1) = psi(t) + v(t)/Lf* delta(t)*dt
v(t+1) = v(t) + a(t)*dt
```

where `Lf` is constant value equals to distance between front axis and gravity center; dt - time difference between time points `t+1` and `t`.

Also we have two equations for control error:

```
cte(t+1) = f(x(t)) - y(t) + v(t)*sin(epsi(t))*dt
epsi(t+1) = psi(t) - psiDes(t) + v(t)/Lf *delta(t)*dt);
```

where `cte` - cross track error of the car; epsi(t) - orientation error of the car; f(x) - function of desired trajectory; psiDes - desired car orientation.

### Timestep Length and Elapsed Duration

Here I will discuss how I've chosen `N` and `dt` parameters of prediction procedure. `dt` is time difference between prediction steps and `N` is number of prediction steps.

Big value of `dt` may produce errors due to nonlinearity effects, from other side small value may ineffectively increase computation time. I've experimented for values from 0.5 to 0.1 and chosen the last one.

The same situation with `N` - too big value may increase computation cost and make solver take into account too far points on the track, which really doesn't matter. Too small value may exclude close enough points from consideration which also may be bad from control point of view. I've tried values from 5 to 30 and chosen value 10 as the best experiment result.

### Polynomial Fitting and MPC Preprocessing

We receive reference trajectory as a set of points in local coordinate system. First of all we convert this points to car coordinate system (file `main.cpp` line `120`) and then fit this points to `polyfit` function to estimate polyline of fixed order (file `main.cpp` line `122`). I have found that 2th order is fine fore this project.

### Model Predictive Control with Latency

Here I'll describe two most valuable parts of tuning MPC - cost function and latency issue.

#### Cost function

My final variant of cost function looks this way:

```
	for (unsigned int t=0; t<_mpcI->_N; ++t) {
	    fg[0] += CppAD::pow(vars[_mpcI->_cte_start + t], 2); // lower cross track error
	    fg[0] += 200*CppAD::pow(vars[_mpcI->_epsi_start + t], 2); // lower track angle error
	    fg[0] += 0.02*CppAD::pow(vars[_mpcI->_v_start + t] - _mpcI->_desiredV, 2); // reach desired speed
	}
	for (int t = 0; t < _mpcI->_N - 1; t++) {
	    fg[0] += 1000*CppAD::pow(vars[_mpcI->_delta_start + t], 2); // minimize control signals
// 	    fg[0] += 0.01*CppAD::pow(vars[_mpcI->_a_start + t], 2); // minimize control signals
	}
	for (int t = 0; t < _mpcI->_N - 2; t++) {
	    fg[0] += 100*CppAD::pow(vars[_mpcI->_delta_start + t + 1] - vars[_mpcI->_delta_start + t], 2); // make control signals smooth
// 	    fg[0] += 0.01*CppAD::pow(vars[_mpcI->_a_start + t + 1] - vars[_mpcI->_a_start + t], 2); // make control signals smooth
	}
```

Even though car can turn sharp it is better not to do so. That's why I've chosen high weight for part of cost function responsible for steering angle. Also it helped solver to avoid some strange trajectories.

Then I've increased weight of `epsi` error which helped the car to move over the curves more smooth. This way I was able to increase car speed up to 80 mp/h.

Last thing, I've removed acceleration parts (we are in the racing car on the track, by the way, it is not good to limit it's acceleration) and decreased weight of target speed. That allowed me to set target speed to 100 mp/h and allowed car to break before the turns and accelerate on the straight part.

#### Dealing with latency

The control signal is send to simulator with latency of 0.1 second. So it is a good idea to predict where car will be in 0.1 second and send control signal for this point in time. I do it in function `PredictState` (file `MPC.cpp`, lines `241-262`). It uses model to predict car state after dt with input values of current state and controls.

