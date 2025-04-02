function T = fkine(S, M, q, frame)
    % FKINE Computes the forward kinematics of a serial robot manipulator.
    %   This function calculates the pose (homogeneous transformation matrix) of the
    %   end-effector of a serial robot manipulator given its joint screw axes, the
    %   home configuration, the joint angles, and the desired reference frame 
    %   ('space' or 'body') for the output transformation.
    
    %   Syntax:
    %       T = fkine(S, M, q, frame)
    
    %   Input Arguments:
    %       S - A 6xN matrix of screw axes for each joint in the manipulator's
    %           base frame, where N is the number of joints. Each column of S
    %           represents a screw axis in the format: [omega; v] where omega
    %           is the angular velocity vector and v is the linear velocity
    %           vector of the screw axis.
    %       M - The 4x4 home configuration matrix of the manipulator. This
    %           matrix represents the pose of the end-effector when all the
    %           joint variables are zero.
    %       q - A 1xN vector of the manipulator's joint variables (angles for
    %           revolute joints, displacements for prismatic joints).
    %       frame - A string specifying the reference frame for the output
    %               transformation. It can be 'space' for the space frame or
    %               'body' for the body frame.
    
    %   Output Argument:
    %       T - The 4x4 homogeneous transformation matrix representing the pose
    %           of the robot's end-effector in the specified frame.

    % Number of joints.
    n_joints = size(q, 2);
    
    % For each joint:
    for i=1:n_joints
        
        % Calculate i-th Transformation matrix.
        Transformation = twist2ht(S(:,i),q(i));
        
        % Initialize T.
        if i == 1
            T = Transformation;
            
        % Update T.
        else        
            T = T * Transformation;
        end
    end 
    
    % Multiply by home configuration.
    if strcmp(frame, 'space')
        T = T * M;
        
    elseif strcmp(frame, 'body')
        T = M * T; 
        
    end
end
