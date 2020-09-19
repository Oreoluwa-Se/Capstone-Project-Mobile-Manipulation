# Capstone-Project-Mobile-Manipulation
* This repo is for the Capstone project from the [Mordern Robotics: Mechanics, Planning, and Control Specialization](https://www.coursera.org/specializations/modernrobotics) on coursera.
* The goal is to [move and plan]() the trajectory for the end-effector on a 5R robot arm articulated on a four mecanum wheels mobile base to move a cube from one to another specific location.
* Details of the capstone project can be found [here](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone)

![Initial_image](http://hades.mech.northwestern.edu/images/3/33/Yb-book.png)

# Project Directory

# How To Use:
1. The code works with an '\mr' folder that can be [downloaded](https://github.com/NxRLab/ModernRobotics) and should be put in the root folder as shown above. The folder should be added to the path.
2. Run the 'main.mlx' which is heavily commented if interested. The script generates:
	* A file used to simulare the robot motion in *CoppeliaSim* called 'Path.csv'
	* An 'Xerr.csv' file that documents the end effector error at given instances.
3. Upon execution, the *user* can either select preset values, or have acess to change the following paramenters:
	* Controller gain: Proportional [Kp], Integral [Ki]
	* Chassis parameters: Chasis start configuration, Initial and Final cube locations
	* Gripper parameters:
		* Initial gripper state 1-closed, 0-open
		* Number of trajectories per second [k]
		* Motion type 1 = Screw motion, 2 = Cartesian 
		* Time scaling 3 = Cubic, 5 = Quintic
	* Other parameters include:
		* Maximum wheel speed
		* Maximum joint speed
		* dt-time step
		* Total simulation time

# Results:
The best way I found in manipulating the controller gain is to slowly increase the proportional gain till the system shows signs of over shoot. As shown below:
<img src="./result/Overshoot/Overshoot.gif" alt="SystemOvershoot" style="zoom: 100%;"/>