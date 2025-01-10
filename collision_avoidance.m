function QP_norm = collision_avoidance(p1,p2, V1, V2,V3,V4,V5,V6,pt,TE ,q,d,a,alpha) 
    % The values in input are:
    % p1, p2 the two points of the segment that has minimum distance, one
    % is on the robot link and one is on the human arm link
    % V1,V2,V3, distances links robot-humans from compute_distance
    % pt is the target point
    % TE is the end effector current position
    % dh prameters for the robot

disp('STARTING COLLISION AVOIDANCE *************************************************');
% calcolo la differenza tra questi due punti, quindi la distanza più
%piccola che c'è tra braccio e robot

d0 = (p1' - p2') /1000; % distanza robot e braccio minima
dg = (pt - TE) /1000; %distanza tool e target position
de = (TE - p2) /1000; % distanza tool e il punto di collisione


%vettori delle distanze minime ottenute dall'algoritmo di distanza
d01 = (V1(1,:) - V1(2,:)) /1000;      
d02 = (V2(1,:) - V2(2,:)) /1000;
d03 = (V3(1,:) - V3(2,:)) /1000; 
d04 = (V4(1,:) - V4(2,:)) /1000;
d05 = (V5(1,:) - V5(2,:)) /1000;
d06 = (V6(1,:) - V6(2,:)) /1000; %sarebbe de

modd01 = norm(d01) ;     % lunghezze
modd02 = norm(d02) ;
modd03 = norm(d03) ;
modd04 = norm(d04) ;
modd05 = norm(d05) ;
modd06 = norm(d06) ;

%definisco i moduli dei campi potenziali
modde = norm(de); %/1000;
modd0 = norm(d0); %/1000;
moddg = norm(dg); %/1000;

wx=0;
wy=0;
wz=0;

% definisco la direzione 
dgvers = dg/norm(moddg); %versore dal tool al goal
devers_1 = d01'/modd01; % versore che va dal link 1 all'ostacolo
devers_2 = d02'/modd02; % versore che va dal link 2 all'ostacolo
devers_3 = d03'/modd03; % versore che va dal link 3 all'ostacolo
devers_4 = d04'/modd04;
devers_5 = d05'/modd05;

  
% v_rep = 1;%6; %[m/s]   Sono le velocità dei campi potenziali
dist_min_des=0.15;
% v_rep = 1;
% v_att = v_rep/dist_min_des;%4; %[m/s]
v_att = 6;
v_rep = 1;

%definisco la velocità attrattiva sul tool
Vatt = v_att*dgvers;   %scalare * versore


%definisco le velocità repulsive su ogni link
Vrep1 = v_rep*devers_1/(modd01);

Vrep2 = v_rep*devers_2/(modd02);

Vrep3 = v_rep*devers_3/(modd03);
Vrep4 = v_rep*devers_4/(modd04);
Vrep5 = v_rep*devers_5/(modd05);



%%matrici di interesse
%   ROBOT
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

%% calcolo dello jacobiano del punto più vicino (p1)

%coseni direttori zi                      % centri delle 6 terne        
z(:,1) = A01(1:3,3);                    O(:,1) = A01(1:3,4);           % NB vett colonna
z(:,2) = A02(1:3,3);                    O(:,2) = A02(1:3,4);
z(:,3) = A03(1:3,3);                    O(:,3) = A03(1:3,4);
z(:,4) = A04(1:3,3);                    O(:,4) = A04(1:3,4);
z(:,5) = A05(1:3,3);                    O(:,5) = A05(1:3,4);
z(:,6) = A06(1:3,3);                    O(:,6) = A06(1:3,4);  % ci metto la punta del tool

disp('CONTINUING 1 COLLISION AVOIDANCE *************************************************');
%% Jacobiano di ogni punto vicino dei link 1,2,3.

% jacobiano link 1

   k1 = 1;
    for  i = 1:k1
    rj1 = V1(1,:);     % coordinata c1 rispetto terna zero robot
 O_diff1 = (rj1' - O(:,i))*0.001;    %divido per mille perche sono passato da [mm]---> [m]: devo trovare tipo i radianti al sec (uso SI)
 J_v1(:,i) = cross(z(:,i),O_diff1);
 J_omega1(:,i) = z(:,i);
    end
    J1 = [J_v1;J_omega1];
   
% else
%     k1 = 0;

%  Jacobiano link 2
k2 = 2;
    for  i = 1:k2
    rj2 = V2(1,:);     % coordinata c1 rispetto terna zero robot
 O_diff2 = (rj2' - O(:,i))*0.001;    %divido per mille perche sono passato da [mm]---> [m]: devo trovare tipo i radianti al sec (uso SI)
 J_v2(:,i) = cross(z(:,i),O_diff2);
 J_omega2(:,i) = z(:,i);
    end
    % mat.J_v2 = J_v2;
    % mat.J_omega2 = J_omega2;
    J2 = [J_v2;J_omega2];
% else
%     k2 = 0;



%  Jacobiano link 3
k3 = 3;
    for  i = 1:k3
    rj3 = V3(1,:);     % coordinata c1 rispetto terna zero robot
 O_diff3 = (rj3' - O(:,i))*0.001;    %divido per mille perche sono passato da [mm]---> [m]: devo trovare tipo i radianti al sec (uso SI)
 J_v3(:,i) = cross(z(:,i),O_diff3);
 J_omega3(:,i) = z(:,i);
    end
    % mat.J_v3 = J_v3;
    % mat.J_omega3 = J_omega3;
    J3 = [J_v3;J_omega3];
% else
%     k3 = 0;

%  Jacobiano link 4
k4 = 4;
    for  i = 1:k4
    rj4 = V4(1,:);     % coordinata c1 rispetto terna zero robot
 O_diff4 = (rj4' - O(:,i))*0.001;    %divido per mille perche sono passato da [mm]---> [m]: devo trovare tipo i radianti al sec (uso SI)
 J_v4(:,i) = cross(z(:,i),O_diff4);
 J_omega4(:,i) = z(:,i);
    end
    % mat.J_v3 = J_v3;
    % mat.J_omega3 = J_omega3;
    J4 = [J_v4;J_omega4];
% else
%     k3 = 0;

%  Jacobiano link 5
k5 = 5;
    for  i = 1:k5
    rj5 = V5(1,:);     % coordinata c1 rispetto terna zero robot
 O_diff5 = (rj5' - O(:,i))*0.001;    %divido per mille perche sono passato da [mm]---> [m]: devo trovare tipo i radianti al sec (uso SI)
 J_v5(:,i) = cross(z(:,i),O_diff5);
 J_omega5(:,i) = z(:,i);
    end
    % mat.J_v3 = J_v3;
    % mat.J_omega3 = J_omega3;
    J5 = [J_v5;J_omega5];
% else
%     k3 = 0;


%  Jacobiano link 6
k6 = 6;
    for  i = 1:k6
    rj6 = V6(1,:);     % coordinata c1 rispetto terna zero robot
 O_diff6 = (rj6' - O(:,i))*0.001;    %divido per mille perche sono passato da [mm]---> [m]: devo trovare tipo i radianti al sec (uso SI)
 J_v6(:,i) = cross(z(:,i),O_diff6);
 J_omega6(:,i) = z(:,i);
    end
    % mat.J_v3 = J_v3;
    % mat.J_omega3 = J_omega3;
    J6 = [J_v6;J_omega6];
% else
%     k3 = 0;


disp('CONTINUING 2 COLLISION AVOIDANCE *************************************************');

Vc_formula = Vatt;
Vcw = [Vc_formula(1);Vc_formula(2);Vc_formula(3);wx;wy;wz];   %[6x1] velcità corretta TOOL

Vc1_formula = Vrep1 ;
Vcw1 = [Vc1_formula(1);Vc1_formula(2);Vc1_formula(3);wx;wy;wz];   %[6x1] velcità corretta punto link 1

Vc2_formula = Vrep2 ;
Vcw2 = [Vc2_formula(1);Vc2_formula(2);Vc2_formula(3);wx;wy;wz];   %[6x1] velcità corretta punto link 2

Vc3_formula = Vrep3 ;
Vcw3 = [Vc3_formula(1);Vc3_formula(2);Vc3_formula(3);wx;wy;wz];   %[6x1] velcità corretta punto link 3

Vc4_formula = Vrep4 ;
Vcw4 = [Vc4_formula(1);Vc4_formula(2);Vc4_formula(3);wx;wy;wz];   %[6x1] velcità corretta punto link 4

Vc5_formula = Vrep5 ;
Vcw5 = [Vc5_formula(1);Vc5_formula(2);Vc5_formula(3);wx;wy;wz];   %[6x1] velcità corretta punto link 5
%% calcolo le q repulsive per i link 

% Link 1
J_in1 = pinv(J1);
qp1 = (J_in1 * (Vcw1)); 

numZerosToAdd1 = 6 - length(qp1);     
% Aggiunta di zeri alla fine del vettore
QP1 = [qp1', zeros(1,numZerosToAdd1)];   % vettore [1x6]   con gli zeri

%Link 2
J_in2 = pinv(J2);
qp2 = (J_in2 * (Vcw2));

numZerosToAdd2 = 6 - length(qp2);     
% Aggiunta di zeri alla fine del vettore
QP2 = [qp2', zeros(1,numZerosToAdd2)];   % vettore [1x6]   con gli zeri

%Link 3
J_in3 = pinv(J3);
qp3 = (J_in3 * (Vcw3));    

numZerosToAdd3 = 6 - length(qp3);     
% Aggiunta di zeri alla fine del vettore
QP3 = [qp3', zeros(1,numZerosToAdd3)];   % vettore [1x6]   con gli zeri

%Link 4
J_in4 = pinv(J4);
qp4 = (J_in4 * (Vcw4));    

numZerosToAdd4 = 6 - length(qp4);     
% Aggiunta di zeri alla fine del vettore
QP4 = [qp4', zeros(1,numZerosToAdd4)];   % vettore [1x6]   con gli zeri

%Link 5
J_in5 = pinv(J5);
qp5 = (J_in5 * (Vcw5));    

numZerosToAdd5 = 6 - length(qp5);     
% Aggiunta di zeri alla fine del vettore
QP5 = [qp5', zeros(1,numZerosToAdd5)];   % vettore [1x6]   con gli zeri


%calcolo le q attrative totali e le q repulsive totali
QP_rep = QP1 + QP2 + QP3 + QP4 + QP5;
% disp('The total repulsive is');
% disp(QP_rep);

J_invt= pinv(J6);
QP_att = J_invt * Vcw; 
% QP_att = 0; 
% disp('The total attractive is:');
% disp(QP_att);

QP = QP_att' + QP_rep;

QP_norm = (QP / norm(QP)) * 0.3; %normalize the velocity

disp('ENDING COLLISION AVOIDANCE *************************************************');

end