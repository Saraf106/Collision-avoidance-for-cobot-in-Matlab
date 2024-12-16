%% This script performs the simulation of the inverse kinematics using a line representation of the robot

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

%% simulation

% define the target configuration
q1 = [pi/3, -2*pi/6, pi/6, 5*pi/6, pi+pi/6, -pi/6];
T06_target = forward_kinematics(q1, d, a, alpha);

%define the starting configuration
% q0 = [pi/4, pi/6, -pi/4, pi/3, -pi/6, pi/2];
q0 = [pi/3, pi/4, -pi/6, pi/2, -pi/3, pi/4];
T06_current = forward_kinematics(q0, d, a, alpha);

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
    
    % Creazione dei punti 
    P0x = 0; P0y = 0; P0z = 0;
    P1x = A01(1,4); P1y = A01(2,4); P1z = A01(3,4);
    P2x = A02(1,4); P2y = A02(2,4); P2z = A02(3,4);
    P3x = A03(1,4); P3y = A03(2,4); P3z = A03(3,4);
    P4x = A04(1,4); P4y = A04(2,4); P4z = A04(3,4);
    P5x = A05(1,4); P5y = A05(2,4); P5z = A05(3,4);
    P6x = A06(1,4); P6y = A06(2,4); P6z = A06(3,4);
    pu=A02*[a(3);0;0;1]; %necessari visto che i frame sono spostati
    pv=A03*[a(4);0;0;1];
    
    %creazione della linea
    l1 = line ([P0x P1x P2x pu(1) P3x pv(1) P4x P5x P6x], [P0y P1y P2y pu(2) P3y pv(2) P4y P5y P6y],[P0z P1z P2z pu(3) P3z pv(3) P4z P5z P6z]);

    % inizializza la figura di visualizzazione
    figure(1);
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('TM5-700 Inverse kinematics');   
    
    frame_handles = [];
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
   
    disp('Updating line');

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
    
    % Creazione dei punti 
    P0x = 0; P0y = 0; P0z = 0;
    P1x = A01(1,4); P1y = A01(2,4); P1z = A01(3,4);
    P2x = A02(1,4); P2y = A02(2,4); P2z = A02(3,4);
    P3x = A03(1,4); P3y = A03(2,4); P3z = A03(3,4);
    P4x = A04(1,4); P4y = A04(2,4); P4z = A04(3,4);
    P5x = A05(1,4); P5y = A05(2,4); P5z = A05(3,4);
    P6x = A06(1,4); P6y = A06(2,4); P6z = A06(3,4);
    pu=A02*[a(3);0;0;1];
    pv=A03*[a(4);0;0;1];

    l1.XData = [P0x P1x P2x pu(1) P3x pv(1) P4x P5x P6x];
    l1.YData = [P0y P1y P2y pu(2) P3y pv(2) P4y P5y P6y];
    l1.ZData = [P0z P1z P2z pu(3) P3z pv(3) P4z P5z P6z];
    
    

    % Display all the joints frames:
    
    % target
    hold on
    plot3(T06_target(1,4),T06_target(2,4),T06_target(3,4),'b*')

    L =35;

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
