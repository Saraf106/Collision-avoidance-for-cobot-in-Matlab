%% Simulation for the real Techman TM5-700, script used for the tetsing phase, 
%% it's the simulation_realtime.m with modifications done during testing and the commands 
%% to give to the robots 
clear
close all
clc
%dbstop if error
addpath('../KinZ-Matlab-master/Mex/');
%% Stabilisco connessione
techman=TM('10.10.10.160', 5890);         
%% 
stop(techman);

%% utility per prendere punti
home = [ 96.8755 , -39.1042,  -57.2958 , -56.5507 , -90.0000   ,      0]; % position between the 2° and 3° position
okay0=PTP(techman,"JPP",home, 40,100,10,false);

%% SIMULAZIONE
%This script simulates the collision avoidance for a 7M5-700 techman cobot with a human arm


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
%q1 = [1.5126  , -1.0235  ,  1.4841   , 0.7455  ,  3.6235 ,   0.4712];
% q1= [ -0.69  ,  -0.52   , 0.78   ,      0  , -1.57    ,     0];
q1 = [-0.70 , -0.88, 0.95, -0.58, 1.57, 0];
T06_target = forward_kinematics(q1, d, a, alpha);

%define the starting configuration


%q0 = [0.4748   ,-0.8883  ,  2.0036  , -0.5838   , 5.5112,    1.2558];
%q0=[0.34  , -0.52,  0.78  ,       0  , -1.57   ,      0];
% q0=[0  , -1.5708 ,  -1.5708  , -1.5708  , -1.5708  ,       0];
% q0 = [0.3491 ,  -2.2689 ,  -0.7854 ,  -1.5708   ,-1.5708   ,      0];
q0 = [0.12 , -0.88, 0.95, -0.58, 1.57, 0];
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
kz = KinZ('720p', 'unbinned', 'wfov', 'imu_off', 'bodyTracking');

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
range = 1200;
view(46,29);
axis([-range range -200 1000 0 range]);


xlabel('X');
ylabel('Y');
zlabel('Z');
title('TM5-700 Inverse kinematics');   
    
frame_handles = [];
l1=[];
color_ssv = rand(1,3);
   
   
% Loop until pressing 'q' on any figure
k=[];
i_ask = 0;
n_cicli = 1000;
i_errore=0;
disp('Press q on any figure to exit')
downSample = 2; % subsample pointcloud
    

while true
  
        disp('Current pose T06');
        disp(T06_current);
        disp('Target pose');
        disp(T06_target);
        %se il robot è abbastanza vicino diciamo che è arrivato. Qui diamo
        %una distanza di 2cm per la traslazione e 0.052 rad per la
        %rotazione
        if abs((round(T06_current,2) - round(T06_target,2)))<=[0.0524 0.0524 0.0524 2;
                0.0524 0.0524 0.0524 2;
                0.0524 0.0524 0.0524 2;
                0 0 0 0]
            disp('Target reached');
            break;
        end
       % If user presses 'q', exit loop
        if ~isempty(k)
            if strcmp(k,'q')
                break;
            elseif strcmp(k,'p')
                pause;
            end
            k = [];
        end
        %dichiariamo qui il valore dei punti, così se non trova un frame
        %valido la camera o se il numero di bodies che trova non è > 0 usa
        %questi valori e in questo modo sarà a distanza molto grande e farà direttamente
        % inverse kinematics senza collisione. Nota: non usare tutti 10000
        % come valori altrimenti non funziona la compute_distance
        shoulder_translated = [10000;10000;10000];
        elbow_translated = [10000;11000;10000];
        wrist_translated = [11000;10000;10000];
        hand_translated = [10000;10000;11000];
            
     validData = kz.getframes('color','depth', 'bodies');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices        
        [depth, depthTimestamp] = kz.getdepth;
        [color, colorTimestamp] = kz.getcolor;
        numBodies = kz.getnumbodies;
        disp('Number of bodies are:');
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
        
        %valori assegnati per non usare il nostro braccio ogni volta
       % shoulder_translated = [688.398391723633; -16.891113281250; 485.546401977539];
       % elbow_translated = [664.744400024414;16.5002441406250;227.754196166992];
       % wrist_translated = [466.065181732178;-69.707031250000;198.149215698242];
       % hand_translated = [386.404785156250;-16.417541503906;203.275993347168];

        if numBodies > 0
            shoulder_point = bodies.Position3d(:,13);
            % disp(shoulder_point);
            elbow_point = bodies.Position3d(:,14);
            wrist_point = bodies.Position3d(:,15);
            hand_point = bodies.Position3d(:,16);
            
             % definiamo la rotation and translation della camera rispetto
             % al robot, la x è uguale per robot e per camera, la z è y e la y è -z.
                R = [1 0 0;
                     0 0 1 ;
                    0 -1 0 ;];
               
                t=[600,-900,400]';
    
                %applico rotazione
                shoulder_rotated = (R * shoulder_point);
                elbow_rotated = (R * elbow_point);
                wrist_rotated = (R * wrist_point);
                hand_rotated = (R * hand_point);
                
                %applico traslazione
                shoulder_translated = shoulder_rotated + t;
                elbow_translated = elbow_rotated +t;
                wrist_translated = wrist_rotated +t;
                hand_translated = hand_rotated +t;
        
           
        end
    end


            q = inverse_kinematics(T06_current, 1, 1, 0);

            % si fa un controllo su q4 e su q5, in modo da limitare la
            % rotazione ad angoli compresi tra [-pi,pi]
            if q(4)> pi
                    q(4) = q(4)-2*pi;
            elseif q(4)< -pi
                    q(4)=q(4)+2*pi;
            end
            if q(5)> pi
                    q(5) = q(5)-2*pi;
            elseif q(5)< -pi
                    q(5)=q(5)+2*pi;
            end

            disp('le q dopo sono:');
            disp(q);
            disp('la T06 aggiornata è:');
            disp(T06_current);

            % check delle distaze di ogni link 
            [D,C1,C2,V1,V2,V3,V4,V5,V6] = compute_distance(q,d,a,alpha,shoulder_translated, elbow_translated, wrist_translated, hand_translated);

            pt = T06_target(1:3,4);
            Te = T06_current(1:3,4);
            disp('The distances are:');
            disp(D);

            %check per la collision avoidance, controlliamo che:
            % 1) se almeno una distanza è minore di 300  &&
            % 2) se almeno una coordinata tra x, y, z è maggiore di 50
            % (vogliamo che quando è abbastanza vicino al target ci vada
            % con l'inverse_kinematics direttamente
            % fa la collision avoidance
            if any(D<300, 'all') && any(abs((round(T06_current(1:3,4),2) - round(T06_target(1:3,4),2))) > [50;50;50],'all')
                
                QP = collision_avoidance(C1,C2,V1,V2,V3,V4,V5,V6,pt,Te,q,d,a,alpha);  %ritorna le q punto
            
                % dalle q punto ottengo le nuove q da dare per evitare
                % l'ostacolo usando la formula sotto. 
                % NOTA: se le velocità QP vicine allo zero, significa che il robot 
                % è sufficientemente vicino alla posizione target q1, quindi
                % lo faccio direttamente andare in q1.

                if abs(round(((QP).*0.001),2)) == [0,0,0,0,0,0]
                    q = q1;
                else
                    q = q + 0.001 * QP;   % q_n = q_o + q_dot * dt, dt è il passo temporale, 
                end


                %%%%%%%%%%%%%%%%%%%
                % Qui viene fatta la tabella di conversione tra le q mie e
                % quelle effettive del robot dato che i frame dei giunti
                % sono disposti diversamente nel robot reale.
                % Faccio la trasformazione perchè devo dare le q al robot
                % quindi gli devo dare le sue corrispondenti
                molt = [1,-1,-1,-1,1,1];
                trans = [pi/2,-pi/2,0,-pi/2,-pi,0];
                q_robot = q.*molt+trans;
                if q_robot(4)> pi
                    q_robot(4) = q_robot(4)-2*pi;
                elseif q_robot(4)< -pi
                    q_robot(4)=q_robot(4)+2*pi;
                end
                if q_robot(5)> pi
                    q_robot(5) = q_robot(5)-2*pi;
                elseif q_robot(5)< -pi
                    q_robot(5)=q_robot(5)+2*pi;
                end
                 for i=1:1:6
                    q_robot(i)=rad2deg(q_robot(i));
                 end
                 
                q_robot(6)=0; %la assegnamo noi perchè fa quello che vuole senza senso

                % diciamo al robot in che punto nuovo andare
                okay0 =PTP(techman,"JPP",q_robot, 40,100,10,false);
                pause(0.5); % matlab aspetta 0.5 secondi che il robot raggiunga la posizione
                % stop(techman);
                %pause(0.2);
                
                %%%%%%%%%%%%%%%%

                %%%%%%%%%%%%%%%%
                % Ask joint, chiediamo le q effettive di dove è il robot,
                % può essere che la simulazione sia avanti rispetto a dove
                % è realmente il robot, quindi chiediamo dove è realmente e
                % la aggiustiamo di conseguenza
                i_ask = i_ask + 1;

                if i_ask == n_cicli % chiediamo il check ogni tot volte, non ad ogni ciclo
                    
                    % faccio un stop and clear buffer per bloccare il robot
                    % dove è
                    stop(techman);
                    
                    pause(0.2);
                    
                    %chiediamo le joint effettive dei robot da fermo
                    joint_real = ask(techman,1);
                    
                    % è necessario mettere in pausa matlab per 5 secondi altrimenti 
                    % il messaggio dal robot a noi non arriva corretto e da errore
                    pause(5); 
                    
                    joint_real=double(joint_real);
                    joint_real_rad=joint_real;
                    
                    for i=1:1:6
                        joint_real_rad(i)=deg2rad(joint_real(i));
                    
                    end
                    
                    q = (joint_real_rad-trans)./molt;
                    
                    disp('Le q reali del techman sono:');
                    disp(joint_real);
                    
                    %calcolo l'errore tra le q nostre e quelle effettive
                    %del robot, l'errore è piccolo quindi non è necessario
                    %fare l'ask
                    i_errore=i_errore+1;
                    q_errore(i_errore) = max(q_robot - joint_real);
                    
                    disp('errore q_robot - joint_real');
                    disp(q_robot - joint_real);

                    q(6)=0; %sempre perchè fa quello che vuole con la sesta joint
                    i_ask = 0;
                end
                q(6)=0;
                %%%%%%%%%

                % calcolo la pose nuova
                T06_current = forward_kinematics(q,d,a,alpha);
            else
                % é il caso dove la collision avoidance non viene fatta

                % fare l'update sul piano delle joint, funziona meglio del
                % piano cartesiano
                q = q + (q1-q).*0.05;

                if abs(round(((q1-q).*0.05),2)) == [0,0,0,0,0,0]
                    q = q + q1-q;
                end

                %%%%%%%%
                molt = [1,-1,-1,-1,1,1];
                trans = [pi/2,-pi/2,0,-pi/2,-pi,0];
                q_robot = q.*molt+trans;
                if q_robot(4)> pi
                    q_robot(4) = q_robot(4)-2*pi;
                elseif q_robot(4)< -pi
                    q_robot(4)=q_robot(4)+2*pi;
                end
                if q_robot(5)> pi
                    q_robot(5) = q_robot(5)-2*pi;
                elseif q_robot(5)< -pi
                    q_robot(5)=q_robot(5)+2*pi;
                end
                 for i=1:1:6
                    q_robot(i)=rad2deg(q_robot(i));
                 end
                 
                okay0 =PTP(techman,"JPP",q_robot, 40,100,10,false);
                pause(1);
                stop(techman);
                %pause(0.2);

                %%%%%%
                %ask joint

                i_ask = i_ask + 1;

               
                if i_ask == n_cicli && any(abs((round(T06_current(1:3,4),2) - round(T06_target(1:3,4),2))) > [50;50;50],'all')
                    stop(techman);
                    pause(0.2);
                    joint_real = ask(techman,1);
                    pause(5);
                    joint_real=double(joint_real);
                    joint_real_rad=joint_real;
                    for i=1:1:6
                        joint_real_rad(i)=deg2rad(joint_real(i));
                     end
                    q = (joint_real_rad-trans)./molt;
                    disp('Le q reali del techman sono:');
                    disp(joint_real);
                    q(6)=0;
                    i_ask = 0;
                end

                
                %%%%%%%%%

                q(6)=0;
                
                T06_current = forward_kinematics(q,d,a,alpha);
            end
           
            if ~isempty(frame_handles)
                delete(frame_handles);
                frame_handles = [];
            end
            
            if ~isempty(l1)
                delete(l1);
      
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
        pause(0.05);
       
 end
