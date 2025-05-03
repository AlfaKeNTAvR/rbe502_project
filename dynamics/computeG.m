function G = computeG(q, load)
% Computes the gravity torque vector G(q) for a 3-DOF robotic arm.
%
% Input:
%   q - Joint positions [q1; q2; q3] (3×1)
%
% Output:
%   G - Gravity torque vector (3×1)


    m3 = 0.25 + load;    
    q2 = q(2);
    q3 = q(3);

    G = [0;
        (2943*cos(q2))/2000 + (2943*m3*cos(q2 + q3))/2000 + (2943*m3*cos(q2))/1000;
        (2943*m3*cos(q2 + q3))/2000];

end