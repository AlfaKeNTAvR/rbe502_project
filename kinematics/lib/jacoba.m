function J_a = jacoba(S, M, q)
    % JACOBA Computes the analytical Jacobian for a serial robot manipulator.
    %   The analytical Jacobian relates the joint velocities to the end-effector
    %   linear and angular velocities in the manipulator's base frame. This function
    %   uses the screw axes S, the home configuration M, and the joint variables q
    %   to compute the analytical Jacobian.
    
    %   Syntax:
    %       J_a = jacoba(S, M, q)
    
    %   Input Arguments:
    %       S - A 6xN matrix of screw axes for each joint in the manipulator's
    %           base frame, where N is the number of joints. Each column of S
    %           represents a screw axis for a joint.
    %       M - The 4x4 home configuration matrix of the manipulator. This matrix
    %           represents the pose of the end-effector when all the joint
    %           variables are zero.
    %       q - A 1xN vector of the manipulator's joint variables (angles for
    %           revolute joints, displacements for prismatic joints).
    
    %   Output Argument:
    %       J_a - The 6xN analytical Jacobian matrix. The first three rows correspond
    %             to the end-effector's angular velocity, and the last three rows
    %             correspond to the end-effector's linear velocity.

    % Jacobian in Space Frame.
    J_s = jacob0(S,q);

    % Analytical Jacobian.
    T = fkine(S,M,q,'space'); 
    J_a = -skew(T(1:3, 4)) * J_s(1:3, :) + J_s(4:6, :);
end