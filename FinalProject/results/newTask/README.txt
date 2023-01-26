Kevin Nella
"newTask" Task README

This task follows a new reference trajectory based on custom initial and final cube configurations as follows:

Initial position: (x,y,theta) = (1 m, -1 m, 0 rad)
Final position: (x,y,theta) = (1 m, 2 m, 0 rad)


The controller implemented for this solution was a Feedforward + Proportional Feedback controller. The Proportional gain Kp was set to 10.0 * 6x6 Identity matrix.