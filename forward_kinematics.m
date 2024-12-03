% This function computes the forward kinematics of the robot given the
% joints variables q1, q2, q3, q4, q5, q6.

function T06 = forward_kinematics(q, d, a, alpha)

        %computes single transformation
        T01 = denavit(q(1),d(1),a(1),alpha(1));
        T12 = denavit(q(2),d(2),a(2),alpha(2));
        T23 = denavit(q(3),d(3),a(3),alpha(3));
        T34 = denavit(q(4),d(4),a(4),alpha(4));
        T45 = denavit(q(5),d(5),a(5),alpha(5));
        T56 = denavit(q(6),d(6),a(6),alpha(6));
        
        % moltiplication
        T06 = T01 * T12 * T23 * T34 * T45 * T56;
     
        
      
end