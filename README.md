# Kuka-Pick-and-Place
## Final Project for ME 449: Robotic Manipulation

## Overview
This package contains a MATLAB script that performs the following:

1. Generates reference trajectory of an 8-segment trajectory of the end effector of a youBot starting at an initial configuration, traveling to and picking up a block, and placing the block at a new location/configuration.

2. Feedback control to mitigate error between actual and reference trajectory by commanding a corrective twist determined by the implementation of feedforward, proportional, integral controls, or any combination of the aforementioned.

3. Determines the next state of the actual trajectory from current configuration and feedback control-commanded twist.

### Best Task

This task follows the reference trajectory for picking and placing a cube with the initial and final locations as mentioned in the project prompt. The controller implemented for this solution was a Feedforward + Proportional Feedback controller. The Proportional gain Kp was set to 3.0 * 6x6 Identity matrix.

### Overshoot Task

This task follows the same reference trajectory as the "best" task. The controller implemented for this solution was a Feedforward + Proportional + Integral Feedback controller. The Proportional gain Kp was set to 5.0 * 6x6 Identity matrix, and the Integral gain Ki was set to 15.0 * 6x6 Identity matrix.

### New Task

This task follows a new reference trajectory based on custom initial and final cube configurations as follows:

Initial position: (x,y,theta) = (1 m, -1 m, 0 rad)
Final position: (x,y,theta) = (1 m, 2 m, 0 rad)

The controller implemented for this solution was a Feedforward + Proportional Feedback controller. The Proportional gain Kp was set to 10.0 * 6x6 Identity matrix.

## Implementation
The results of three different tasks are included in the package: Best, Overshoot, and New Task. These are found in the Results directory within directories of the same names. Further information about each task is given in respective READMEs.

To run each of the simulations, save their respective .csv files (trajectory and Xerror), and plot their Xerror, just hit Run. Everything will be taken care of. To change any gains, time periods, or initial and final locations for newTask - the options are located at the top of the script so they may be easily changed. To run the simulation, load the appropriate .csv into the youBot environment in CoppeliaSim, and press play.

## Video Demonstration

Results of 'Best' PI gains configuration:

https://user-images.githubusercontent.com/58793794/214854660-81fc16f5-a7d7-45fd-8cdb-353ea36fc39c.mp4


Results of 'Overshoot' PI gain configuration:

https://user-images.githubusercontent.com/58793794/214854746-2ed826f7-8cb0-42ad-b58b-e84ede8701e6.mp4


Results of 'New Task' trajectory and PI gain configuration:

https://user-images.githubusercontent.com/58793794/214854793-772a8d59-3efb-4f3c-85c2-de2d6c00d786.mp4


### Disclaimer
This project was not worked on through git - the repository was created after completion.