This repository describes my work done at the University of Padua as a research assistant 
which led to my Master thesis project. 
The goal of the project was to develop a collision avoidance algorithm using Matlab for a cobot, in this case the TECHMAN TM5-700 was used.
The work was done as follows:
- study of the kinematics of the robot
- study of the collision avoidance literature and implementation of a new one based on joint velocities
- implementation of a MATLAB simulation of the robot kinematics and collision avoidance startegy
- integration of the Orbbec Femto Bolt camera used to detect the position of the human, specifically his arm
- testing phase done in the laboratory
To know  more, please read the document: Experimental Validation of a collision avoidance strategy for a collaborative robot.


Truoughout the process of coding I saved the partial steps that can be run separately.
The scripts: compute_distance.m, collision_avoidance.m, disframe.m, denavit.m, drawSSV.m are cannot be run separatly but are called inside the different simulations scripts.
The simualtions are described as follows:
- simulation_line_robot.m -> performs the simulation of the inverse kinematics of the robot where the links are lines
- simulation_SSV_robot.m -> performs the simulation of the inverse kinematics of the robot where the links are modeled with SSVs
- simulation_with_human.m -> performs the simulation of the robot with the SSVs but there is also the integration of the robot arm (both with lines or SSV)
- simulation_robot_human.m -> performs the simulation of the collision avoidance strategy with the robot and the human arm (both with SSVs)
- simulation_realtime.m -> it's the simulatio of the collision avoidance strategy with the integration of the camera, so the human arm is not modeled as a robotic arm as before but 4 points detected by the camera are used as starting point for the creation of the arm.
- TRIAL_WITH_TECHMAN_COLLAVOIDANCE -> it's the script used in the laboratory tests

