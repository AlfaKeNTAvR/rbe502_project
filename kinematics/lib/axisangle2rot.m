function R = axisangle2rot(omega,theta)
    % AXISANGLE2ROT Converts an axis-angle representation to a rotation matrix.
    %   This function computes the rotation matrix corresponding to a rotation
    %   of theta radians about an axis defined by the 3-element vector omega,
    %   using the Rodrigues rotation formula.
    
    %   Syntax:
    %       R = axisangle2rot(omega, theta)
    
    %   Input Arguments:
    %       omega - A 3-element vector representing the axis of rotation.
    %               It does not need to be unit length, but must not be zero.
    %       theta - A scalar representing the rotation angle in radians.
    
    %   Output Argument:
    %       R - The 3x3 rotation matrix corresponding to the specified axis-angle
    %           rotation.
    
    %   Example:
    %       omega = [0; 0; 1]; % Rotation axis (z-axis)
    %       theta = pi/2;      % Rotation angle (90 degrees)
    %       R = axisangle2rot(omega, theta)
    %       % This will compute the rotation matrix for a 90-degree rotation 
    %       % around the z-axis.

    % Calculate omega skew-symmetric omega.
    omega_ssm = skew(omega);
    omega_ssm_squared = omega_ssm * omega_ssm;
    
    % Rodrigues formula.
    R = eye(3) + sin(theta)*omega_ssm + (1-cos(theta))*omega_ssm_squared;
end