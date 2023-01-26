# Kuka-Pick-and-Place
## Final Project for ME 449: Robotic Manipulation

## Overview
This package contains a MATLAB script that performs the following:

1. Generates reference trajectory of an 8-segment trajectory of the end effector of a youBot starting at an initial configuration, traveling to and picking up a block, and placing the block at a new location/configuration.

2. Feedback control to mitigate error between actual and reference trajectory by commanding a corrective twist determined by the implementation of feedforward, proportional, integral controls, or any combination of the aforementioned.

3. Determines the next state of the actual trajectory from current configuration and feedback control-commanded twist.

## Implementation
The results of three different tasks are included in the package: Best, Overshoot, and New Task. These are found in the Results directory within directories of the same names. Further information about each task is given in respective READMEs.

To run each of the simulations, save their respective .csv files (trajectory and Xerror), and plot their Xerror, just hit Run. Everything will be taken care of. To change any gains, time periods, or initial and final locations for newTask - the options are located at the top of the script so they may be easily changed. To run the simulation, load the appropriate .csv into the youBot environment in CoppeliaSim, and press play.

## Video Demonstration

Results of 'Best' PI gains configuration:


Results of 'Overshoot' PI gain configuration:


Results of 'New Task' trajectory and PI gain configuration:





### Disclaimer

