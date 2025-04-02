function T = tdh(theta, d, a, alpha)
% Takes as input the DH parameters associated with the link of a robotic arm and returns the corresponding 4x4 homogeneous transformation matrix.

    r = a;  
    T = [[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) r*cos(theta)]; [sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) r*sin(theta)]; [0 sin(alpha) cos(alpha) d]; [0 0 0 1]];
end