# ORCA_advance_control

A nonlinear model predictive controller for ROV disturbance rejection control. 


## Requirement

This package requires the following software packages
- [control-toolbox](https://github.com/ethz-adrl/control-toolbox)
- [ORCA Control](https://github.com/tsaoyu/ORCA_control) simulation, visulisation

additional packages for extra features:
- [sympy](https://www.sympy.org/en/index.html) symbolic controller model development
- [rqt_multiplot_plugin](https://github.com/ANYbotics/rqt_multiplot_plugin) realtime data plot 

## Usage

Since this is a standard C++ ROS package, you can build it via `catkin build` with `gcc >= 7`.

### Manual controller (requires joystick)
```
roslaunch orca_advance_control manual_control.launch
```


### LQR controller

```
roslaunch orca_advance_control lqr_controller.launch
```
### iLQR controller

```
roslaunch orca_advance_control ilqr_controller.launch
```

### NMPC controller
```
roslaunch orca_advance_control mpc_controller.launch
```

If you not yet feel comfortable with work with high-dimensional MPC problem, I have an introductory repository https://github.com/tsaoyu/ct_example for you.



## Citation

Please cite this paper if you use this code and/or data for your research.

Early access version: https://ieeexplore.ieee.org/document/9180253
(We have made this paper open access, so please have a look, it is free.) 


## Disclaim
The author release the code WITHOUT ANY GUARANTEE. The user takes the sole responsibilities including but not limited to the injure, casualty and other losses.

## Acknowledgement
This work was funded by the EPSRC as part of the UK Robotics and Artificial Intelligence Hub for Offshore Energy Asset Integrity
Management (ORCA-Hub) under grant EP/R026173/1. 

