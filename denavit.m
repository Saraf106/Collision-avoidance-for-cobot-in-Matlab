% This function return the transformation matrix given the associated dh
% parameters

function dh=denavit(teta, d, a, alfa)

dh = [     cos(teta)             -sin(teta)           0               a     ;
      sin(teta)*cos(alfa)    cos(teta)*cos(alfa)  -sin(alfa)    -d*sin(alfa);
      sin(teta)*sin(alfa)    cos(teta)*sin(alfa)   cos(alfa)     d*cos(alfa);
               0                     0                 0             1       ];
end