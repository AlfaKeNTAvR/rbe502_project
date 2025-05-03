function M = computeM(q, load)
% Computes the joint-space inertia matrix M(q) for a 3-DOF robotic arm.
%
% Input:
%   q - Joint positions [q1; q2; q3] (3×1)
%
% Output:
%   M - Inertia matrix (3×3)
    
    m3 = 0.25 + load; 
    q2 = q(2); 
    q3 = q(3);
    
    % M = [(9*cos(2*q2 + q3))/800 + (27*cos(2*q2))/800 + (9*cos(2*q2 + 2*q3))/3200 + (9*cos(q3))/800 + 117/3200, 0, 0;
    %     0, (9*cos(q3))/400 + 117/1600, (9*cos(q3))/800 + 9/1600;
    %     0, (9*cos(q3))/800 + 9/1600, 9/1600];

    M = [(9*m3)/160 + (9*cos(2*q2))/400 + (9*m3*cos(2*q2))/200 + (9*m3*cos(2*q2 + 2*q3))/800 + (9*m3*cos(q3))/200 + (9*m3*cos(2*q2 + q3))/200 + 9/400, 0, 0;
        0, (9*m3)/80 + (9*m3*cos(q3))/100 + 9/200, (9*m3*(2*cos(q3) + 1))/400;
        0, (9*m3*(20*cos(q3) + 10))/4000, (9*m3)/400];
         
        
end