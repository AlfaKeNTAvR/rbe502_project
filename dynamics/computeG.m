function G = computeG(q)
% Computes the gravity torque vector G(q) for a 3-DOF robotic arm.
%
% Input:
%   q - Joint positions [q1; q2; q3] (3×1)
%
% Output:
%   G - Gravity torque vector (3×1)
    
    q2 = q(2);
    q3 = q(3);
    
    G = [0;
      (2943*cos(q2 + q3))/8000 + (8829*cos(q2))/4000; 
      (2943*cos(q2 + q3))/8000];
    end