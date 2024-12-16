%% This script defines the robot parameters and performs inverse kinematics, 
%% the forward kinematics is then used to check the correctness of the
%% inverse kinematics
clc
clear all
close all
%DIRECT AND INVERSE KINEMATIC

% The robot considered is a TM5-700 Techman cobot with 6dof non redundand
% with the following parameters in mm:
a3 = 329;
a4 = 311.50;

d1 = 145.20;
d4 = 106;
d5 = 106;
d6 = 113.15;
d2 = 146;
d3 = -129.70;
        
% The DH parameters are:
% q = [q1; q2; q3; q4; q5; q6];   
d = [d1, d2, d3, d4, d5, d6];
a = [0, 0, a3, a4, 0, 0];  
alpha = [0, -pi/2, 0, 0, -pi/2, -pi/2]; 

%% compute forward kinematics

%q = [pi/3, -2*pi/6, pi/6, 5*pi/6, pi+pi/6, -pi/6]; %sinistra, up, no flip
%q = [0,0,0,0,0,0];
q = [pi/4, pi/6, -pi/4, pi/3, -pi/6, pi/2];
T06 = forward_kinematics(q, d, a, alpha);

disp('The pose of the end effector is:');
disp(T06);


%% compute inverse kinematics

q = inverse_kinematics(T06, 1, 1, 1);

disp('The joints are:');
disp(q);

disp('The pose of the end effector is:');
disp(forward_kinematics(q, d, a, alpha));