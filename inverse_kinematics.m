%% this function computes the inverse kinematics of the robot using geometrical approach

function q = inverse_kinematics(T06, shoulder_left, elbow_up, no_flip)
  
%definition of the robot parameters
a3 = 329;
a4 = 311.50;
d1 = 145.20;
d4 = 106;
d5 = 106;
d6 = 113.15;

d2 =146;
d3 = -129.70;

%define the dh parameters

d = [d1; d2; d3; d4; d5; d6];
a = [0; 0; a3; a4; 0; 0];
alpha = [0; -pi/2; 0; 0; -pi/2; -pi/2];


P05 = T06 * [ 0 0 -d6 1]';

if shoulder_left == 1

    q1 = atan2(P05(2), P05(1)) - asin((d2 -abs(d3)+d4)/sqrt(P05(1)^2 + P05(2)^2));

    T01 = denavit(q1,d(1),a(1), alpha(1));
    T16 = inv(T01) * T06;
    P16 = T16 *[0 0 0 1]';

    T61 = inv(T16);
    yx = T61(1,2);
    yy = T61(2,2);

    if no_flip == 1
        q5 = pi + acos((P16(2)-d2+abs(d3)-d4)/d6);

        q6 = - atan2(yy,yx);


    else
        q5 = -pi -acos((P16(2)-d2+abs(d3)-d4)/d6);
        q6 = - pi - atan2(yy,yx);
    end

        T45 = denavit(q5,d(5),a(5),alpha(5));
        T56 = denavit(q6,d(6), a(6),alpha(6));
        T01 = denavit(q1,d(1), a(1),alpha(1)); %not sure about this
        T16 = inv(T01) * T06;

        T14 = T16 * (inv(T45 * T56));
        P14 = T14 * [ 0 0 0 1]';

        P14xz = sqrt(P14(1)^2 + P14(3)^2);

    if elbow_up == 1

        q3 = pi - acos((-P14(1)^2 - P14(3)^2 +a4^2 + a3^2)/(2 *a4*a3));
        q2 = -atan2(P14(3), P14(1)) - acos((+a3^2 -a4^2+P14(1)^2 + P14(3)^2)/(2*a3*P14xz));
    else
        q3 = - pi + acos((-P14(1)^2 - P14(3)^2 +a4^2 + a3^2)/(2 *a4*a3));

        q2 = -atan2(P14(3), P14(1)) + acos((-a4^2 +a3^2+P14(1)^2 + P14(3)^2)/(2*a3*P14xz));
    end

    T12 = denavit(q2,d(2), a(2),alpha(2));


    T23 = denavit(q3,d(3), a(3),alpha(3));

    T34 = inv(T12 * T23) * T14;
    %P34 = T34(1:3, 1:3);
    xy = T34(2,1);
    xx = T34(1,1);

    q4 = atan2(xy,xx);
else

    q1 = 3/2 * pi - acos(d2 -abs(d3)+d4/(sqrt(P05(1)^2 + P05(2)^2)) ) + atan2(P05(2), P05(1));

    T01 = denavit(q1,d(1), a(1),alpha(1));
    T16 = inv(T01) * T06;
    P16 = T16*[0 0 0 1]';
    P16y = P16(2);
    T61 = inv(T16);
    yx = T61(2,2);
    yy = T61(1,2);


    q5 = - acos((-P16y+d2-abs(d3)+d4)/d6);
    q6 = - atan2(yy,yx);


    T45 = denavit(q5,d(5), a(5),alpha(5));
        T56 = denavit(q6,d(6), a(6),alpha(6));
        T01 = denavit(q1,d(1), a(1),alpha(1)); %not sure about this
        T16 = inv(T01) * T06;
        T14 = T16 * inv(T45 * T56);
        P14 = T14 * [ 0 0 0 1]';
        P14xz = sqrt(P14(1)^2 + P14(3)^2);

    if elbow_up == 1

        q3 = pi - acos((- P14(1)^2 - P14(3)^2 + a(3)^2 + a(4)^2)/(2 *a4*a3));
        q2 = -atan2(P14(3), P14(1)) - acos((-a4^2 +a3^2+P14(3)^2 + P14(1)^2)/(2*a3*P14xz));
    else
        q3 = - pi + acos((- P14(1)^2 - P14(3)^2 + a(3)^2 + a(4)^2)/(2 *a4*a3));
        q2 = -atan2(P14(3), P14(1)) + acos((-a4^2 +a3^2+ P14(3)^2 + P14(1)^2)/(2*a3*P14xz));
    end

    T12 = denavit(q2,d(2), a(2),alpha(2));
    T23 = denavit(q3,d(3), a(3),alpha(3));
    T34 = inv(T12 * T23) * T14;
    xy = T34(2,1);
    xx = T34(1,1);
    q4 = atan2(xy,xx);
end

q = [q1, q2, q3, q4, q5, q6]; 

end