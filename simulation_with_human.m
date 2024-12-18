%% This script simulates the inverse kinematics of the techman cobot, with a
%% human arm in the space, no collision avoidance is implemented so the
%% collision happens.
clc
clear all
close all

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

% The human arm considered has the following parameters:

%quello che funziona ma non dovrebbe
% u_d = [200, 380, 200, 50];
% u_a = [100, 0, 0, 0];
% u_alpha = [0, -90, -100, 0];
% u_q = [0, 0, 0, 0];

%disposizione del braccio dalla parte opposta del robot
u_d = [100, 0, 0, 0];
u_a = [100, 200, 100, 100];
u_alpha = [0, -pi/2, 0, 0];
u_q = [pi/2, 0, 0, 0];


%% Simulation of the robot with SSV 
% define the target configuration
q1 = [pi/3, -2*pi/6, pi/6, 5*pi/6, pi+pi/6, -pi/6];
T06_target = forward_kinematics(q1, d, a, alpha);

%define the starting configuration
% q0 = [pi/4, pi/6, -pi/4, pi/3, -pi/6, pi/2];
q0 = [pi/3, pi/4, -pi/6, pi/2, -pi/3, pi/4];
T06_current = forward_kinematics(q0, d, a, alpha);


ax = axes;
ax_u.Position = [100, 800, 0, 0];
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');

model = "TM5-700";
% carica file modello
data = load(fullfile(pwd,model));
            
% Creo struttura dati del robot e carico i dati
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

Links = [d1, d2, d3, d4, d5, d6];

% u_links = [u_d1, -300, -100, 50];

u_links = [100, 200, 200, 100];

% Create transform hierarchy
robot.ha = hgtransform('Parent', ax);
robot.h0 = hgtransform('Parent', robot.ha);
robot.hLink = gobjects(1, robot.n+2);
robot.hLink(1) = hgtransform('Parent', robot.h0);

for i = 2:robot.n
    robot.hLink(i) = hgtransform('Parent', robot.hLink(i-1));
end

% Create the human arm structure
human_arm = struct();
human_arm.hAx = ax;
human_arm.n = 4; % gradi di libertà
human_arm.SSV = [];

% Create transform hierarchy
human_arm.ha = hgtransform('Parent', ax);
human_arm.h0 = hgtransform('Parent', human_arm.ha);
human_arm.hLink = gobjects(1, human_arm.n+2);
human_arm.hLink(1) = hgtransform('Parent', human_arm.h0);

for i = 2:human_arm.n
    human_arm.hLink(i) = hgtransform('Parent', human_arm.hLink(i-1));
end

n = 50;
 % Matrici di trasformazione:
    A01 = denavit(q0(1), d(1), a(1), alpha(1));
    A12 = denavit(q0(2), d(2), a(2), alpha(2));
    A23 = denavit(q0(3), d(3), a(3), alpha(3));
    A34 = denavit(q0(4), d(4), a(4), alpha(4));
    A45 = denavit(q0(5), d(5), a(5), alpha(5));
    A56 = denavit(q0(6), d(6), a(6), alpha(6));
    
    % Matrici di trasformazione dal sistema zero a ogni joint:  
    A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;
    A06 = A05 * A56;

    set(robot.hLink(1), 'Matrix', A01);
    set(robot.hLink(2), 'Matrix', A12);
    
    set(robot.hLink(3), 'Matrix', A23);
    
    set(robot.hLink(4), 'Matrix', A34);
   
    set(robot.hLink(5), 'Matrix', A45);
    
    set(robot.hLink(6), 'Matrix', A56);


    % stessa cosa per il braccio umano
    A0_u0 = [1, 0, 0, 100;
                           0, 1, 0, 0;
                           0, 0, 1, 0;
                           0, 0, 0, 1];
    A01_u = denavit(u_q(1), u_d(1), u_a(1), u_alpha(1));
    A12_u = denavit(u_q(2), u_d(2), u_a(2), u_alpha(2));
    A23_u = denavit(u_q(3), u_d(3), u_a(3), u_alpha(3));
    A34_u = denavit(u_q(4), u_d(4), u_a(4), u_alpha(4));
    
    
    % Matrici di trasformazione dal sistema zero a ogni joint:  

    A02_u = A01_u * A12_u;
    A03_u = A02_u * A23_u;
    A04_u = A03_u * A34_u;
   

    set(human_arm.hLink(1), 'Matrix', A01_u);
    set(human_arm.hLink(2), 'Matrix', A12_u);
    
    set(human_arm.hLink(3), 'Matrix', A23_u);
    
    set(human_arm.hLink(4), 'Matrix', A34_u);



    %prova con linee
    % Creazione dei punti 
    P0x = 100; P0y = 0; P0z = 0;
    P1x = A01_u(1,4); P1y = A01_u(2,4); P1z = A01_u(3,4);
    P2x = A02_u(1,4); P2y = A02_u(2,4); P2z = A02_u(3,4);
    P3x = A03_u(1,4); P3y = A03_u(2,4); P3z = A03_u(3,4);
    P4x = A04_u(1,4); P4y = A04_u(2,4); P4z = A04_u(3,4);
    
    l1 = line ([P0x P1x P2x P3x P4x], [P0y P1y P2y P3y P4y],[P0z P1z P2z P3z P4z]);
    % inizializza la figura di visualizzazione
    figure(1);
    grid on;
    zoom on;
    rotate3d on
    axis equal;
    range = 700;
    view(46,29);
    axis([-range range -200 1000 -range range]);

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('TM5-700 Inverse kinematics');   
    
    frame_handles = [];
    color = rand(1,3);
    color_u = rand(1,3);
    
 
for i= 1:n
   
  
    % Position interpolation (translation part)
    p_start = T06_current(1:3, 4);  % Current position
    p_end = T06_target(1:3, 4);      % Target position
    p_current = p_start + (p_end - p_start) * (i/n);
    
    % Orientation interpolation (rotation part)
    % You might want to use quaternion interpolation (SLERP) for better results
    R_start = T06_current(1:3, 1:3);  % Current rotation matrix
    R_end = T06_target(1:3, 1:3);      % Target rotation matrix
    
    % Simple linear interpolation of rotation matrices
    R_current = R_start + (R_end - R_start) * (i/n);
   
    
    % Construct interpolated transformation matrix
    T_current = eye(4);
    T_current(1:3, 1:3) = R_current;
    T_current(1:3, 4) = p_current;

    q = inverse_kinematics(T_current, 1, 1, 1);

    if ~isempty(frame_handles)
        delete(frame_handles);
        frame_handles = [];
    end

    disp('The new angles are:');
    disp(q);
   
    disp('Updating SSV');

     % Matrici di trasformazione:
    A01 = denavit(q(1), d(1), a(1), alpha(1));
    A12 = denavit(q(2), d(2), a(2), alpha(2));
    A23 = denavit(q(3), d(3), a(3), alpha(3));
    A34 = denavit(q(4), d(4), a(4), alpha(4));
    A45 = denavit(q(5), d(5), a(5), alpha(5));
    A56 = denavit(q(6), d(6), a(6), alpha(6));
    
    
    % Matrici di trasformazione dal sistema zero a ogni joint:  
    A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;
    A06 = A05 * A56;

    set(robot.hLink(1), 'Matrix', A01);
    set(robot.hLink(2), 'Matrix', A12);
    
    set(robot.hLink(3), 'Matrix', A23);
    
    set(robot.hLink(4), 'Matrix', A34);
   
    set(robot.hLink(5), 'Matrix', A45);
    
    set(robot.hLink(6), 'Matrix', A56);
    
    % Creazione dei punti 
    % Draw SSV for each link
    h = gobjects(1, robot.n);
    u_h = gobjects(1, human_arm.n);
    for j = 1:robot.n

    
        if j == 2
            robot.SSV(j).R = [25,25]; 
            robot.SSV(j).L = [-d2,-a3];        % Length
            robot.SSV(j).rot(:, :, 1) = eye(4);      % No rotation
            robot.SSV(j).shift(:, :, 1) = eye(4);    % No shift
            % Segment 2 (aligned with X-axis)
            robot.SSV(j).rot(:, :, 2) = [0, 0, -1, 0;
                           0, 1, 0, 0;
                           1, 0, 0, 0;
                           0, 0, 0, 1];
    
            robot.SSV(j).shift(:, :, 2) =  eye(4); 
        elseif j == 3
            robot.SSV(j).R = [25,25]; 
            robot.SSV(j).L = [-d3,-a4];        % Length
            robot.SSV(j).rot(:, :, 1) = eye(4);      % No rotation
            robot.SSV(j).shift(:, :, 1) = eye(4);    % No shift
            % Segment 2 (aligned with X-axis)
            robot.SSV(j).rot(:, :, 2) = [0, 0, -1, 0;
                           0, 1, 0, 0;
                           1, 0, 0, 0;
                           0, 0, 0, 1];
            
            robot.SSV(j).shift(:, :, 2) =  eye(4);   
        else
    
            robot.SSV(j).R = 25; 
            robot.SSV(j).L = -Links(j);        % Length
            robot.SSV(j).rot = eye(4);   % Rotation matrix
            robot.SSV(j).shift = eye(4); % Shift matrix
        end
    
        if j <= length(robot.SSV)

            drawSSV(robot, j, 'points', 40, ...
                'FaceColor', color, ...  % Random color for each link
                'FaceAlpha', 0.7, ...       
                'EdgeColor', color);
            hold on

        end

        % Uncomment this for cicle to have the SSV also for the human arm
        % for k=1:human_arm.n
        % 
        %         if k~=1
        %              human_arm.SSV(k).R = 25; 
        %                 human_arm.SSV(k).L = -u_links(k);    
        %                 human_arm.SSV(k).shift = eye(4);
        %                 human_arm.SSV(k).rot = [
        %                 0, 0, 1, 0;
        %                 0, 1, 0, 0;
        %                 -1, 0, 0, 0;
        %                 0, 0, 0, 1
        %             ];
        %                 % drawSSV(human_arm, k, 'points', 40, ...
        %                 %     'FaceColor', color_u, ...  % Random color for each link
        %                 %     'FaceAlpha', 0.7, ...       
        %                 %     'EdgeColor', color_u);
        % 
        % 
        %         else
        %                 human_arm.SSV(k).R = 25; 
        %                 human_arm.SSV(k).L = -u_links(k);    
        %                 human_arm.SSV(k).shift = eye(4);
        %                 human_arm.SSV(k).rot = eye(4);
        %         end
        % 
        %         drawSSV(human_arm, k, 'points', 40, ...
        %                     'FaceColor', color_u, ...  % Random color for each link
        %                     'FaceAlpha', 0.7, ...       
        %                     'EdgeColor', color_u);
        %             hold on
        % 
        %  end
   

    end

    
    
    

    % Display all the joints frames:
 
    %target
    hold on
    plot3(T06_target(1,4),T06_target(2,4),T06_target(3,4),'b*')

    L =50;

    frame_handles = [frame_handles; disframe(eye(4), L, 'o')]; 
    frame_handles = [frame_handles; disframe(A0_u0, L, 'o')]; 
    frame_handles = [frame_handles; disframe(A01_u, L, 'none')];    
    frame_handles = [frame_handles; disframe(A02_u, L, 'none')];   
    frame_handles = [frame_handles; disframe(A03_u, L, 'none')];
    frame_handles = [frame_handles; disframe(A04_u, L, 'none')];
    % frame_handles = [frame_handles; disframe(A05, L, 'none')];
    % frame_handles = [frame_handles; disframe(A06, L, 'v')];
    % 
    drawnow;
    pause(0.1);
end