%% Full symbolic inertial matrix.
M = [(9*cos(2*q2 + q3))/800 + (27*cos(2*q2))/800 + (9*cos(2*q2 + 2*q3))/3200 + (9*cos(q3))/800 + 117/3200, 0, 0;
    0, (9*cos(q3))/400 + 117/1600, (9*cos(q3))/800 + 9/1600;
    0, (9*cos(q3))/800 + 9/1600, 9/1600];
 
%% Full symbolic Coriolis matrix.
C = [0;
    (9*q1_d^2*(4*sin(2*q2 + q3) + 12*sin(2*q2) + sin(2*q2 + 2*q3)))/3200;
    (9*q1_d^2*sin(q3))/1600 + (9*q2_d^2*sin(q3))/800 + (9*q1_d^2*sin(2*q2 + q3))/1600 + (9*q1_d^2*sin(2*q2 + 2*q3))/3200 + (9*q2_d*q3_d*sin(q3))/800];
 
%% Full symbolic gravity vector.
G = [0;
    (2943*cos(q2 + q3))/8000 + (8829*cos(q2))/4000; 
    (2943*cos(q2 + q3))/8000];
                                             

                      