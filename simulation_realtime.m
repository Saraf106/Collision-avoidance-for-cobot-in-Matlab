%% This script simulates the collision avoidance for a 7M5-700 techman cobot with a human arm
clc
clear all
close all

addpath('../KinZ-Matlab-master/Mex/');

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


%% Collision avoidance algorithm

% define the target configuration
% q1 = [pi/3, -2*pi/6, pi/6, 5*pi/6, pi+pi/6, -pi/6];
q1 = [1.5126  , -1.0235  ,  1.4841   , 0.7455  ,  3.6235 ,   0.4712];

T06_target = forward_kinematics(q1, d, a, alpha);

%define the starting configuration
% q0 = [pi/4, pi/6, -pi/4, pi/3, -pi/6, pi/2];
%q0 = [pi/3, pi/4, -pi/6, pi/2, -pi/3, pi/4];
%q0 = [1.0472 ,  -0.2900  ,  1.1709   , 0.9821   , 4.9603 , 0.6159 ];
%q0 = [1.0472  , -0.1067 ,   1.0060  ,  0.9514  ,  5.0615, 0.6853];
q0 = [0.4748   ,-0.8883  ,  2.0036  , -0.5838   , 5.5112,    1.2558];
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
robot.n = length(data.dati.jHome); % gradi di libertà
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


% Create transform hierarchy
robot.ha = hgtransform('Parent', ax);
robot.h0 = hgtransform('Parent', robot.ha);
robot.hLink = gobjects(1, robot.n+2);
robot.hLink(1) = hgtransform('Parent', robot.h0);

for i = 2:robot.n
    robot.hLink(i) = hgtransform('Parent', robot.hLink(i-1));
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

% Takes info from the camera
kz = KinZ('720p', 'binned', 'nfov', 'imu_off', 'bodyTracking');

% images sizes
% images sizes
depthWidth = kz.DepthWidth; 
depthHeight = kz.DepthHeight; 
outOfRange = 2000;
colorWidth = kz.ColorWidth; 
colorHeight = kz.ColorHeight;

% Color image is to big, let's scale it down
colorScale = 1;

% Create matrices for the images
depth = zeros(depthHeight,depthWidth,'uint16');
color = zeros(colorHeight*colorScale,colorWidth*colorScale,3,'uint8');

% depth stream figure
e.h = figure;
e.ax = axes;
e.im = imshow(depth,[0 outOfRange]);
title(e.ax, 'Depth Source')
colormap(e.ax, 'Jet')
colorbar(e.ax)
set(e.h,'keypress','k=get(e.h,''currentchar'');'); % listen keypress

% color stream figure
c.h = figure;
c.ax = axes;
c.im = imshow(color,[]);
title(c.ax, 'Color Source (press q to exit)');
set(c.h,'keypress','k=get(c.h,''currentchar'');'); % listen keypress


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
color_ssv = rand(1,3);
   
   
% Loop until pressing 'q' on any figure
k=[];

disp('Press q on any figure to exit')
downSample = 2; % subsample pointcloud
    
% while true
%     % Get frames from Kinect and save them on underlying buffer
%     % 'color','depth','infrared'
%     validData = kz.getframes('color','depth', 'bodies');
% 
%     % Before processing the data, we need to make sure that a valid
%     % frame was acquired.
%     if validData
%         % Copy data to Matlab matrices        
%         [depth, depthTimestamp] = kz.getdepth;
%         [color, colorTimestamp] = kz.getcolor;
%         numBodies = kz.getnumbodies;
%         disp(numBodies)
%         bodies = kz.getbodies();    
% 
%         % update depth figure
%         e.im = imshow(depth, 'Parent', e.ax);
% 
%         % update color figure
%         color = imresize(color,colorScale);
%         c.im = imshow(color, 'Parent', c.ax);
% 
%         % Draw bodies on depth image
%         kz.drawbodies(e.ax,bodies,'depth',2, 1);
% 
%         % Draw bodies on color image
%         kz.drawbodies(c.ax,bodies,'color',2, 1);
% 
%         if numBodies > 0 
%             shoulder_point = bodies.Position3d(:,13);
%             disp(shoulder_point);
%             elbow_point = bodies.Position3d(:,14);
%             wrist_point = bodies.Position3d(:,15);
%             hand_point = bodies.Position3d(:,16);
% 
%                 R = [0 0 -1;
%                     1 0 0 ;
%                     0 -1 0 ;
%                     ];
%                 t = [850 200 50]';
% 
%                 shoulder_rotated = (R * shoulder_point);
%                 elbow_rotated = (R * elbow_point);
%                 wrist_rotated = (R * wrist_point);
%                 hand_rotated = (R * hand_point);
% 
%                 shoulder_translated = shoulder_rotated + t;
%                 elbow_translated = elbow_rotated +t;
%                 wrist_translated = wrist_rotated +t;
%                 hand_translated = hand_rotated +t;
% 
% 
%         end
% 
% 
% 
%     end

    %     % If user presses 'q', exit loop
    % if ~isempty(k)
    %     if strcmp(k,'q'); break; end
    %     k = [];
    % end
  
%     pause(0.01)
% end

while true
  
        disp('Current pose T06');
        disp(T06_current);
        disp('Target pose');
        disp(T06_target);
        if T06_current(1:3,4) == T06_target(1:3,4)
            disp('Target reached');
            break;
        end
        % in questo modo sarà a distanza molto grande e farà direttamente
        % inverse kinematics
        shoulder_translated = [10000;10000;10000];
        elbow_translated = [10000;10000;10000];
        wrist_translated = [10000;10000;10000];
        hand_translated = [10000;10000;10000];
            
     validData = kz.getframes('color','depth', 'bodies');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices        
        [depth, depthTimestamp] = kz.getdepth;
        [color, colorTimestamp] = kz.getcolor;
        numBodies = kz.getnumbodies;
        disp(numBodies)
        bodies = kz.getbodies();    
       
        % update depth figure
        e.im = imshow(depth, 'Parent', e.ax);

        % update color figure
        color = imresize(color,colorScale);
        c.im = imshow(color, 'Parent', c.ax);
        
        % Draw bodies on depth image
        kz.drawbodies(e.ax,bodies,'depth',2, 1);
        
        % Draw bodies on color image
        kz.drawbodies(c.ax,bodies,'color',2, 1);

        if numBodies > 0 
            shoulder_point = bodies.Position3d(:,13);
            disp(shoulder_point);
            elbow_point = bodies.Position3d(:,14);
            wrist_point = bodies.Position3d(:,15);
            hand_point = bodies.Position3d(:,16);
    
                R = [0 0 -1;
                    1 0 0 ;
                    0 -1 0 ;
                    ];
                t = [850 200 50]';
    
                shoulder_rotated = (R * shoulder_point);
                elbow_rotated = (R * elbow_point);
                wrist_rotated = (R * wrist_point);
                hand_rotated = (R * hand_point);
                
                shoulder_translated = shoulder_rotated + t;
                elbow_translated = elbow_rotated +t;
                wrist_translated = wrist_rotated +t;
                hand_translated = hand_rotated +t;
        
           
        end
    end


            q = inverse_kinematics(T06_current, 1, 1, 1);

            % check delle distaze di ogni link 
            [D,C1,C2,V1,V2,V3,V4,V5,V6] = compute_distance(q,d,a,alpha,shoulder_translated, elbow_translated, wrist_translated, hand_translated);

            pt = T06_target(1:3,4);
            Te = T06_current(1:3,4);
            disp('The distances are:');
            disp(D);
            if any(D<100, 'all')
                QP = collision_avoidance(C1,C2,V1,V2,V3,V4,V5,V6,pt,Te,q,d,a,alpha);  % da sviluppare
                q = q + 0.5 * QP;             % q_n = q_o + q_dot * dt
                T06_current = forward_kinematics(q,d,a,alpha);
            else

                
                 
                p_start = T06_current(1:3, 4);  % Current position
                p_end = T06_target(1:3, 4);     % Target position
                p_current = p_start + (p_end - p_start) * 0.05;
    
                % Orientation interpolation (rotation part)
                % You might want to use quaternion interpolation (SLERP) for better results
                R_start = T06_current(1:3, 1:3);  % Current rotation matrix
                R_end = T06_target(1:3, 1:3);      % Target rotation matrix
    
                % Simple linear interpolation of rotation matrices
                R_current = R_start + (R_end - R_start) * 0.05;
    
    
                % Construct interpolated transformation matrix
                T06_current = eye(4);
                T06_current(1:3, 1:3) = R_current;
                T06_current(1:3, 4) = p_current;

                q = inverse_kinematics(T06_current, 1, 1, 1);
                % T06_current = forward_kinematics(q,d,a,alpha);
            end
            % T06_current = forward_kinematics(q,d,a,alpha);
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
                        'FaceColor', color_ssv, ...  % Random color for each link
                        'FaceAlpha', 0.7, ...       
                        'EdgeColor', color_ssv);
                    hold on

                end


            end



        % Display all the joints frames of robot and human arm:
        figure(1)
        %target
        L =50;
        hold on
        plot3(T06_target(1,4),T06_target(2,4),T06_target(3,4),'b*')
        
        % Combine the points into X, Y, Z vectors
        X = [shoulder_translated(1), elbow_translated(1), wrist_translated(1), hand_translated(1)];
        Y = [shoulder_translated(2), elbow_translated(2), wrist_translated(2), hand_translated(2)];
        Z = [shoulder_translated(3), elbow_translated(3), wrist_translated(3), hand_translated(3)];
        
        % Plot the line connecting the points
        l1 = line(X, Y, Z, 'Color', 'r', 'LineWidth', 2);
        hold on


        frame_handles = [frame_handles; disframe(eye(4), L, 'o')]; 
        frame_handles = [frame_handles; disframe(A01, L, 'none')];    
        frame_handles = [frame_handles; disframe(A02, L, 'none')];   
        frame_handles = [frame_handles; disframe(A03, L, 'none')];
        frame_handles = [frame_handles; disframe(A04, L, 'none')];
        frame_handles = [frame_handles; disframe(A05, L, 'none')];
        frame_handles = [frame_handles; disframe(A06, L, 'v')];



        drawnow;
        pause(0.5);
       
  end