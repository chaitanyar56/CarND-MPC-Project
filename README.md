# Model Predictive Control

The goal of this project is to track the generated trajectory from the path planning module using model predictive control algorithm to steer and accelerate the vehicle on a simulator track. CppAD and Ipopt packages are used to solve the model predictive control problem.

## Model

Vehicle model used in the project is kinematic bicycle model. The equations for the model are shown below.

![alt text](/images/vehicle_model.JPG)

The state  `[x,y,psi,v,cte,epsi]` consist of  x-y position, heading angle, cross track error, and error in heading direction respectively. Actuator `[delta, a]` are steering angle and throttle values with constraints [-25,25] degrees and [-1,1] respectively.

## Time step Length and duration

`N` time step Length and `dt` time step duration allows to predict till `t = Ndt` sec ahead with `N` points. In this project `N = 10` and `dt = 0.1` were optimal enough for prediction with less computations when compared to `N = 25` and `dt = 0.05` which had twice the computations for the same time `t`. 

## Preprocessing and polynomial fitting

Before polynomial fitting the trajectory waypoints needed to be converted to the vehicles co-ordinate system(refer main.cpp line 120). Velocity is converted to meter/s from miles/h to make sure the units of co-ordinate system match. Third degree polynomial is used to fit transformed waypoints.

## Latency
To include the latency effect, the next state of the vehicle for the period of latency is predicted  with the current actuator values and used to solve the model predictive control problem (refer main.cpp line 137).



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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
