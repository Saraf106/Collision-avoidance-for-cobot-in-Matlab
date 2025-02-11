%% this script contains the function to compute the distance between robot and human arm

function [D,C1,C2,V1,V2,V3,V4,V5,V6] = compute_distance(q,d,a,alpha,shoulder_translated, elbow_translated, wrist_translated, hand_translated)

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


P1 = [0,0,0];
Q1 = [A01(1,4), A01(2,4), A01(3,4)];

P2 = [A01(1,4), A01(2,4), A01(3,4)];
Q2 = [A02(1,4), A02(2,4), A02(3,4)];

P3 = [A02(1,4), A02(2,4), A02(3,4)];
Q3 = [A03(1,4), A03(2,4), A03(3,4)];

P4 = [A03(1,4), A03(2,4), A03(3,4)];
Q4 = [A04(1,4), A04(2,4), A04(3,4)];

P5 = [A04(1,4), A04(2,4), A04(3,4)];
Q5 = [A05(1,4), A05(2,4), A05(3,4)];

P6 = [A05(1,4), A05(2,4), A05(3,4)];
Q6 = [A06(1,4), A06(2,4), A06(3,4)];

v1 = Q1-P1;
v2 = Q2-P2;
v3 = Q3-P3;
v4 = Q4-P4;
v5 = Q5-P5;
v6 = Q6-P6;

%sostituire con i punti dal pointcloud

   

P1_u = shoulder_translated';
Q1_u = elbow_translated';

P2_u = elbow_translated';
Q2_u  = wrist_translated';

P3_u = wrist_translated';
Q3_u = hand_translated';

v1U = Q1_u - P1_u;
v2U = Q2_u - P2_u;
v3U = Q3_u - P3_u;

P = [P1;P2;P3;P4;P5;P6]; PU = [P1_u; P2_u; P3_u];
v = [v1;v2;v3;v4;v5;v6]; vU = [v1U;v2U;v3U];

%% LINK 3 ROBOT CONFRONTATO CON TUTTI LINK UMANO
for i = 1:6 % robot
    for j = 1:3 %umano
r = P(i,:)-PU(j,:); %vettore da p. iniz  S1 a p. iniz. S2
a = dot(v(i,:),v(i,:)); %lunghezza quadratica segmento link 3 robot (dot = prod scalare)
e = dot(vU(j,:),vU(j,:)); %lungh. quadratica segmento link 3 UMANO
f = dot(vU(j,:),r); 

c = dot(v(i,:),r);
b = dot(v(i,:),vU(j,:));
denom = a*e - b*b; %sempre positivo
if denom ~= 0   %diverso da 0
            s = (b*f - c*e)/denom;
            s = Clamp(s);
        else 
            s=0;

 end
        t = (b*s+f)/e;
        if t<0
            t = 0;
            s = -c/a;
             s = Clamp(s);
        elseif t>1
            t=1;
            s = (b-c)/a;
            s = Clamp(s);
        end

c1 = P(i,:) + v(i,:)*s;
c2 = PU(j,:) + vU(j,:)*t;
D(i,j) = sqrt(dot(c1-c2,c1-c2));   % MATRICE DISTANZE RELATIVE    % righe = robot, colonne = umano
S(i,j) = s;           %matrici contenenti s e t delle rispettive distanze in D.
T(i,j) = t;        %      sono 18 distanze, 18 t e 18 s
    end
end
% 
% disp(D);
% disp(S);
% disp(T);

%Cerco la distanza con valore più piccolo
min_value_dist = min(D(:)); 
%cerco la posizione della minima distanza trovata così che so a quale link
%del robot, e umano corrisponde
% ottengo la coppia di link più vicina tra tutte
[link, U_link] = find(D == min_value_dist); % (rig = link robot, col = link umano)

% guardo per ogni link del robot quale è la minima distanza con quale link
% umano,    prendo i corrispettivi link robot e umani
min_dist_1 = min(D(1,:));             [link1, U_link1] = find(D == min_dist_1);
min_dist_2 = min(D(2,:));             [link2, U_link2] = find(D == min_dist_2);
min_dist_3 = min(D(3,:));             [link3, U_link3] = find(D == min_dist_3);
min_dist_4 = min(D(4,:));             [link4, U_link4] = find(D == min_dist_4);
min_dist_5 = min(D(5,:));             [link5, U_link5] = find(D == min_dist_5);
min_dist_6 = min(D(6,:));             [link6, U_link6] = find(D == min_dist_6);

% per il più vicino
if length(link) > 1    % questi checks vengono fatti nel caso i vari find() trovino più di un valore
    link = link(2);
end
if length(U_link) > 1
    U_link = U_link(2);
end
 C2 = PU(U_link,:) + vU(U_link,:)*T(link,U_link);
 C1 = P(link,:) + v(link,:)*S(link,U_link);

% per il link 1
if length(link1) > 1
    link1 = link1(2);
end
if length(U_link1) > 1
    U_link1 = U_link1(2);
end
 C2_1 = PU(U_link1,:) + vU(U_link1,:)*T(link1,U_link1);
 C1_1 = P(link1,:) + v(link1,:)*S(link1,U_link1);
V1 = [C1_1;C2_1];


%per il link 2
if length(link2) > 1 
    link2 = link2(2);
end
if length(U_link2) > 1
    U_link2 = U_link2(2);
end
 C2_2 = PU(U_link2,:) + vU(U_link2,:)*T(link2,U_link2);
 C1_2 = P(link2,:) + v(link2,:)*S(link2,U_link2);
V2 = [C1_2;C2_2];


 % per il link 3
if length(link3) > 1
    link3 = link3(2);
end
if length(U_link3) > 1
    U_link3 = U_link3(2);
end

 C2_3 = PU(U_link3,:) + vU(U_link3,:)*T(link3,U_link3);
 C1_3 = P(link3,:) + v(link3,:)*S(link3,U_link3);
 V3 = [C1_3;C2_3];

 %per il link4
if length(link4) > 1
    link4 = link4(2);
end
if length(U_link4) > 1
    U_link4 = U_link4(2);
end

 C2_4 = PU(U_link4,:) + vU(U_link4,:)*T(link4,U_link4);
 C1_4 = P(link4,:) + v(link4,:)*S(link4,U_link4);
 V4 = [C1_4;C2_4];


 %per il link5
if length(link5) > 1
    link5 = link5(2);
end
if length(U_link5) > 1
    U_link5 = U_link5(2);
end

 C2_5 = PU(U_link5,:) + vU(U_link5,:)*T(link5,U_link5);
 C1_5 = P(link5,:) + v(link5,:)*S(link5,U_link5);
 V5 = [C1_5;C2_5];


%per il link6
if length(link6) > 1
    link6 = link6(2);
end
if length(U_link6) > 1
    U_link6 = U_link6(2);
end

 C2_6 = PU(U_link6,:) + vU(U_link6,:)*T(link6,U_link6);
 C1_6 = P(link6,:) + v(link6,:)*S(link6,U_link6);
 V6 = [C1_6;C2_6];

%la funzione ritorna le distanze e tutti i punti di distanze minime trovati


end



function v = Clamp(v)  %clampando, limito il valore di s e t tra 0 e 1
if v < 0 
    v = 0;
elseif v > 1 
    v = 1;
end
end