function J = jacob0(S,q) 
    % JACOB0 Computes the space Jacobian for a serial robot manipulator.
    %   This function calculates the space Jacobian matrix J for a serial robot 
    %   manipulator based on the provided screw axes S in the space frame and 
    %   the joint variables q.
    
    %   Syntax:
    %       J = jacob0(S, q)
    
    %   Input Arguments:
    %       S - A 6xN matrix of screw axes for each joint in the space frame,
    %           where N is the number of joints.
    %           Each column of S represents a screw axis in the format:
    %           [omega; v] where omega is the angular velocity vector and
    %           v is the linear velocity vector of the screw axis.
    %       q - A 1xN vector of joint variables (angles for revolute joints,
    %           displacements for prismatic joints).
    
    %   Output Argument:
    %       J - The 6xN space Jacobian matrix of the robot.
    
    %   Example:
    %       S = [0,0,1,0,0,0; 0,1,0,-1,0,0; 0,0,1,0,-1,0]';
    %       q = [pi/4, pi/3, pi/6];
    %       J = jacob0(S, q)
    %       % This will compute the space Jacobian matrix for the robot with
    %       % given screw axes and joint angles. 
    %       J = [0   -0.7071    0.6124;
    %            0    0.7071    0.6124;
    %            1.0  0         0.5000;
    %            0   -0.7071    0.0947;
    %            0   -0.7071   -0.0947;
    %            0         0         0]

    % Number of joints.
    n_joints = size(q, 2);
    
    % Initialize the Jacobian and Adjoint transformation.
    J = zeros(6, n_joints);
    Adt = zeros(6, 6, n_joints-1);
    
    % Jacobian vector for the 1st joint.
    J(:,1) = S(:,1);
    
    % Jacobian vectors for i=2..n_joints joints.
    for i=2:n_joints
        T = twist2ht(S(:,i-1), q(i-1));
        Adt(:,:,i-1) = adjoint(T);       
        Adt_final = Adt(:,:,1);
       
        % Calculate the final Adjoint transformation.
        for j=2:i-1           
            Adt_final = Adt_final * Adt(:,:,j);
            
        end         
        
        % Jacobian vector for i-th joint.
        J(:,i) = Adt_final * S(:, i);
    end
end