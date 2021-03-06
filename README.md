# robot-navigation

This repository is an implementation of the paper _**Prediction-Correction Interior-Point Method for Time-Varying Convex Optimization**_ written by **Mahyar Fazlyab et al.**.

_**tl;dr: download any of folders (1.1, 2.0, 3.0); open with MATLAB; type main() to see output (graph).**_

As the very eponymous title suggests, this paper goes on to build an interior-point optimization method which is suited for problems whose objective function and constraints are time-varying in nature. In brief, the approach taken combines knowledge of second order dynamics to predict variations with a version of Newton's method to correct the trajectory and converge towards the optimal solution. Hence the name 'prediction-correction'.

The optimization method derived in this paper is then put to use through a few examples, one of which is a robot navigation problem.

Key **assumptions**:
* the robot is spherical with a known radius;
* all obstacles are spherical with known radii and coordinates;
* there is sufficient room between any two obstacles for the robot to go through;
* the robot's initial position is away from any obstacle.

Main stages of **implementation**:
1. v1.1 - 2D workspace with fixed objective;
2. v2.0 - 2D workspace with time-varying objective;
3. v3.0 - 3D workspace with time-varying objective.

Note that the third implementation step was done to look pretty but is essentially the same as the second step.

Plot **legend**:
* x_c: center of mass of initial robot position
* x_d: initial position of objective
* blue line: robot trajectory
* red line: goal trajectory
* green line: projected objective trajectory

**NOTE** For time-varying stages of implementation (2, 3) this is a non-deterministic program as the time is based on the host computer's clock, and the longer the program takes to run, the longer the goal's path will be.

The input variables to main() are hard-coded so that each program can be run without any knowledge of the code.

Below are a few images showing what to expect. First is a figure from the aforementioned paper, followed by the outputs of code versions 1.1, 2.0, and 3.0 respectively.

![](/images/paper_plot.JPG)
![](/images/fixed_2d_plot.JPG)
![](/images/moving_2d_plot.JPG)
![](/images/moving_3d_plot.JPG)
