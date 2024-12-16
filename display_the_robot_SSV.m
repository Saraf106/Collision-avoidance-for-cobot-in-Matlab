%% This script displays the robot TM5-700 (techman) using SSV

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

%% Displaying the robot with SSV 

figure(1)
ax = axes;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');

model = "TM5-700";
            
% carica file modello
data = load(fullfile(pwd,model));
            
% Creo struttura dati e carico i dati
robot = struct();

robot.hAx = ax;
robot.model = data.model;
robot.type = data.type;
robot.n = length(data.dati.jHome); % gradi di libert�
robot.dati = data.dati; % dati modello cinematico
robot.datadir = pwd;
robot.SSV = [];
            
% colori
if isfield(data,'color')
   robot.color = data.color;
end
            
% SSV
if isfield(data,'SSV')
   robot.SSV = data.SSV;
   
end

q_target = [pi/3, -2*pi/6, pi/6, 5*pi/6, pi+pi/6, -pi/6];
L = [d1, d2, d3,d4,d5,d6];

% Create transform hierarchy
robot.ha = hgtransform('Parent', ax);
robot.h0 = hgtransform('Parent', robot.ha);
robot.hLink = gobjects(1, robot.n + 2);
robot.hLink(1) = hgtransform('Parent', robot.h0);

for i = 2:robot.n
    robot.hLink(i) = hgtransform('Parent', robot.hLink(i-1));
end


  % Matrici di trasformazione:
    A01 = denavit(q_target(1), d(1), a(1), alpha(1));
    A12 = denavit(q_target(2), d(2), a(2), alpha(2));
    A23 = denavit(q_target(3), d(3), a(3), alpha(3));
    A34 = denavit(q_target(4), d(4), a(4), alpha(4));
    A45 = denavit(q_target(5), d(5), a(5), alpha(5));
    A56 = denavit(q_target(6), d(6), a(6), alpha(6));

    set(robot.hLink(1), 'Matrix', A01);
    set(robot.hLink(2), 'Matrix', A12);
    
    set(robot.hLink(3), 'Matrix', A23);
    
    set(robot.hLink(4), 'Matrix', A34);
   
    set(robot.hLink(5), 'Matrix', A45);
    
    set(robot.hLink(6), 'Matrix', A56);
  
  

% Draw SSV for each link
h = gobjects(1, robot.n);
for i = 1:robot.n


    if i == 2
        robot.SSV(i).R = [10,10]; 
        robot.SSV(i).L = [-d2,-a3];        % Length
        robot.SSV(i).rot(:, :, 1) = eye(4);      % No rotation
        robot.SSV(i).shift(:, :, 1) = eye(4);    % No shift
        % Segment 2 (aligned with X-axis)
        robot.SSV(i).rot(:, :, 2) = [0, 0, -1, 0;
                       0, 1, 0, 0;
                       1, 0, 0, 0;
                       0, 0, 0, 1];

        robot.SSV(i).shift(:, :, 2) =  eye(4); 
    elseif i == 3
        robot.SSV(i).R = [10,10]; 
        robot.SSV(i).L = [-d3,-a4];        % Length
        robot.SSV(i).rot(:, :, 1) = eye(4);      % No rotation
        robot.SSV(i).shift(:, :, 1) = eye(4);    % No shift
        % Segment 2 (aligned with X-axis)
        robot.SSV(i).rot(:, :, 2) = [0, 0, -1, 0;
                       0, 1, 0, 0;
                       1, 0, 0, 0;
                       0, 0, 0, 1];
        
        robot.SSV(3).shift(:, :, 2) =  eye(4);   
    else

    robot.SSV(i).R = 10; 
    robot.SSV(i).L = -L(i);        % Length
    robot.SSV(i).rot = eye(4);   % Rotation matrix
    robot.SSV(i).shift = eye(4); % Shift matrix
    end

    if i <= length(robot.SSV)
        drawSSV(robot, i, 'points', 40, ...
            'FaceColor', rand(1,3), ...  % Random color for each link
            'FaceAlpha', 0.3, ...        % More transparent to see overlaps
            'EdgeColor', 'black');
        hold on
    end
end

