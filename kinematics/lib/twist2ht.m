function T = twist2ht(S,theta)
    % TWIST2HT Converts a twist vector and a rotation angle into a homogeneous transformation matrix.
    %   This function takes a twist vector S and a scalar theta and computes
    %   the corresponding homogeneous transformation matrix. The twist vector
    %   represents a screw motion in space, combining rotation about and
    %   translation along a screw axis.
    
    %   Syntax:
    %       T = twist2ht(S, theta)
    
    %   Input Arguments:
    %       S - A 6-element vector where the first 3 elements represent the
    %           angular velocity (omega) and the last 3 elements represent the
    %           linear velocity (v).
    %       theta - A scalar representing the rotation angle in radians.
    
    %   Output Argument:
    %       T - The 4x4 homogeneous transformation matrix representing the
    %           specified twist and rotation.
    
    %   Example:
    %       S = [1; 2; 3; 4; 5; 6];
    %       theta = pi/4;
    %       T = twist2ht(S, theta)
    %       % This will compute the homogeneous transformation matrix for the
    %       % given twist vector and angle.
    %       T = [-2.8076   -1.5355    2.2929    0.3839;
    %             2.7071   -1.9289    1.0503    5.2146;
    %            -0.5355    2.4645   -0.4645    4.7732;
    %             0         0         0         1.0000]


    % Angular and linear vectors.
    omega = S(1:3);
    v = S(4:6);

    % Skew-symmetric omega.
    omega_ssm = skew(omega);
    omega_ssm_squared = omega_ssm * omega_ssm;
    
    % Calculate Rotation matrix using Rodrigues formula.
    R = axisangle2rot(omega,theta);

    % Calculate translation vector.
    p = (theta * eye(3) + (1-cos(theta))*omega_ssm + (theta - sin(theta))*omega_ssm_squared)*v;
    
    % Combine R and p into T.
    T = [R, p; 0, 0, 0, 1];
end