function C = computeC(q, qd, load)
% Computes the Coriolis and centrifugal vector for a 3-DOF arm.
%
% Inputs:
%   q   - Joint positions [q1; q2; q3] (3×1)
%   qd  - Joint velocities [q1_dot; q2_dot; q3_dot] (3×1)
%
% Output:
%   C   - Coriolis and centrifugal vector (3×1)

    m3 = 0.25 + load; 
    q2 = q(2); 
    q3 = q(3);
    q1_d = qd(1); 
    q2_d = qd(2); 
    q3_d = qd(3);
    
    % C = [0;
    %     (9*q1_d^2*(4*sin(2*q2 + q3) + 12*sin(2*q2) + sin(2*q2 + 2*q3)))/3200;
    %     (9*q1_d^2*sin(q3))/1600 + (9*q2_d^2*sin(q3))/800 + (9*q1_d^2*sin(2*q2 + q3))/1600 + (9*q1_d^2*sin(2*q2 + 2*q3))/3200 + (9*q2_d*q3_d*sin(q3))/800];
    
    C = [0;
        (9*q1_d^2*(2*sin(2*q2) + 4*m3*sin(2*q2) + m3*sin(2*q2 + 2*q3) + 4*m3*sin(2*q2 + q3)))/800;
        (9*m3*(2*q1_d^2*sin(q3) + 4*q2_d^2*sin(q3) + 2*q1_d^2*sin(2*q2 + q3) + q1_d^2*sin(2*q2 + 2*q3) + 4*q2_d*q3_d*sin(q3)))/800];
 
end