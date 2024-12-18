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
%q = [pi/4, pi/6, -pi/4, pi/3, -pi/6, pi/2];
q = [0.6845 ,  -0.9204    ,2.1654,   -0.5669    ,5.6844   , 1.4441];
T06 = forward_kinematics(q, d, a, alpha);

disp('The pose of the end effector is:');
disp(T06);


%% compute inverse kinematics
% T06 = [0.5000   -0.0795    0.8624  300.3273;
%     0.5000   -0.7866   -0.3624  200.7057;
%     0.7071    0.6124   -0.3536  -10.6358;
%          0         0         0    1.0000];

T06= [-0.4040    0.2667   -0.8750  -200.6870;
    0.1663    0.9620    0.2165  382.2339;
    0.8995   -0.0580   -0.4330  200.8770;
         0         0         0    1.0000];
q = inverse_kinematics(T06, 1, 1, 1);

disp('The joints are:');
disp(q);

disp('The pose of the end effector is:');
disp(forward_kinematics(q, d, a, alpha));