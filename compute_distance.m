%% this script contains the function to compute the distance between robot and human arm

function [D,C1,C2,V1,V2,V3] = compute_distance(q,d,a,alpha,q_u,d_u,a_u,alpha_u)

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

%computes the point of the segments
% Robot
P1 = [A01(1,4), A01(2,4), A01(3,4)];  %shoulder
Q1 = [A03(1,4), A03(2,4), A03(3,4)];

P2 = [A03(1,4), A03(2,4), A03(3,4)];  %elbow
Q2 = [A04(1,4), A04(2,4), A04(3,4)];

P3 = [A04(1,4), A04(2,4), A04(3,4)];  %wrist
Q3 = [A06(1,4), A06(2,4), A06(3,4)];

v1 = Q1-P1;
v2 = Q2-P2;
v3 = Q3-P3;

% HUMAN ARM

    A01_u = denavit(q_u(1), d_u(1), a_u(1), alpha_u(1));
    A12_u = denavit(q_u(2), d_u(2), a_u(2), alpha_u(2));
    A23_u = denavit(q_u(3), d_u(3), a_u(3), alpha_u(3));
    A34_u = denavit(q_u(4), d_u(4), a_u(4), alpha_u(4));
   

% Matrici di trasformazione dal sistema zero a ogni joint:  
    A02_u = A01_u * A12_u;
    A03_u = A02_u * A23_u;
    A04_u = A03_u * A34_u;
    

% Human arm
P1_u = [A01_u(1,4), A01_u(2,4), A01_u(3,4)];  %shoulder
Q1_u = [A02_u(1,4), A02_u(2,4), A02_u(3,4)];

P2_u = [A02_u(1,4), A02_u(2,4), A02_u(3,4)];  %elbow
Q2_u = [A03_u(1,4), A03_u(2,4), A03_u(3,4)];

P3_u = [A03_u(1,4), A03_u(2,4), A03_u(3,4)];  %wrist
Q3_u = [A04_u(1,4), A04_u(2,4), A04_u(3,4)];

v1U = Q1_u -P1_u;
v2U = Q2_u - P2_u;
v3U = Q3_u - P3_u;

P = [P1;P2;P3]; PU = [P1_u; P2_u; P3_u];
v = [v1;v2;v3]; vU = [v1U;v2U;v3U];

%% LINK 3 ROBOT CONFRONTATO CON TUTTI LINK UMANO
for i = 1:3 % robot
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
T(i,j) = t;        %      sono 9 distanze, 9 t e 9 s
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


%la funzione ritorna le distanze e tutti i punti di distanze minime trovati


end



function v = Clamp(v)  %clampando, limito il valore di s e t tra 0 e 1
if v < 0 
    v = 0;
elseif v > 1 
    v = 1;
end
end