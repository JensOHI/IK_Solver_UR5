# Inverse Kinematics Solver for a UR Robot (MATLAB/SIMULINK).
This is a **Inverse Kinematics Solver** for a UR robot and is based on the paper [Kinematics of a UR5](http://rasmusan.blog.aau.dk/files/ur5_kinematics.pdf) by Rasmus Skovgaard Andersen. 

The implemented DH parameters are for the UR5 robot, but can easily be changed to other DH parameters for another UR robot (this has not been tested). The [DH parameter](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics) used.

Three inputs are given to the function:
- **Position** (pos): is a vector containing the desired x-, y-, and z-coordinate for the end-effector.
- **Orientation** (eul): is a vector containing the desired orientation of the end-effector in euler angles (ZYX).
- **Previous joint angles** (qPrevious): is the current configuration of the robot. This is used to find the best configuration for the desired pose, to minimize the sudden jumps in robot configuration. The best configuration is decided based on a sum squared over the distance between the previous configuration and the new (desired) configuration multiplied with a set for weights for each joint.

The implemention is ready to use in MATLAB and Simulink.
