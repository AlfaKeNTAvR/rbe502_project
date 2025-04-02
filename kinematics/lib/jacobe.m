function J_b = jacobe(S, M, q)
    % JACOBE Computes the body Jacobian for a serial robot manipulator.
    %   This function calculates the body Jacobian matrix J_b, which relates the
    %   joint velocities to the spatial velocities of the robot's end-effector
    %   expressed in the end-effector frame. It uses the screw axes S in the space
    %   frame, the manipulator's home configuration M, and the current joint
    %   variables q to compute the body Jacobian.
    
    %   Syntax:
    %       J_b = jacobe(S, M, q)
    
    %   Input Arguments:
    %       S - A 6xN matrix of screw axes for each joint in the space frame, 
    %           where N is the number of joints. Each column of S represents a 
    %           screw axis as [omega; v], where omega and v are the angular and 
    %           linear velocity vectors of the screw axis, respectively.
    %       M - The 4x4 home configuration matrix of the manipulator, representing
    %           the pose of the end-effector when all the joint variables are zero.
    %       q - A 1xN vector of the manipulator's joint variables (angles for
    %           revolute joints, displacements for prismatic joints).
    
    %   Output Argument:
    %       J_b - The 6xN body Jacobian matrix of the robot. The first three rows 
    %             correspond to the angular velocities, and the last three rows 
    %             correspond to the linear velocities of the end-effector, both 
    %             expressed in the end-effector frame.

    % Tranformation matrix.
    T = fkine(S,M,q,'space');

    % Rotation matrix and transition vector.
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Adjoint transformation matrix from Space to Body.
    Ad_Tbs = [ R'  zeros(3, 3); 
              -R' * skew(p) R']; 
    
    % Jacobian in Space Frame.
    J_s = jacob0(S,q);
    
    % Jacobian in Body Frame.
    J_b = Ad_Tbs * J_s;
end