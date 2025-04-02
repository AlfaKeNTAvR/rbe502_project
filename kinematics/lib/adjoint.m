function Adt = adjoint(T_AB)
    % ADJOINT Computes the adjoint transformation matrix for a given homogeneous transformation.
    %   This function takes a homogeneous transformation matrix T_AB and computes
    %   its adjoint transformation matrix. The adjoint transformation matrix is
    %   useful in converting twist vectors from one coordinate frame to another.
    
    %   Syntax:
    %       Adt = adjoint(T_AB)
    
    %   Input Argument:
    %       T_AB - A 4x4 homogeneous transformation matrix, representing the
    %              transformation from frame A to frame B. It is composed of a
    %              3x3 rotation matrix R and a 3x1 translation vector p.
    
    %   Output Argument:
    %       Adt - The 6x6 adjoint transformation matrix derived from T_AB.
    
    %   Example:
    %       T_AB = [eye(3), [1; 2; 3]; 0, 0, 0, 1];
    %       Adt = adjoint(T_AB)
    %       % This will compute the adjoint transformation matrix for the given
    %       % homogeneous transformation matrix.

    % Get Rotation matrix and Translation vector.
    R = T_AB(1:3, 1:3);
    p = T_AB(1:3, 4);
    
    % Calculate skew-symmetric p.
    p_ssm = skew(p);
    
    % Calculate Adjoint Transformation.
    Adt = [R, zeros(3); p_ssm*R, R];
end

