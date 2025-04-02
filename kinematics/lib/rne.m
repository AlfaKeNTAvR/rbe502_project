function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%

    % Forward iterations
    g = params.g;
    S = params.S;
    M = params.M;
    G = params.G;
    Ftip = params.Ftip;
    jointPos = params.jointPos;
    jointVel = params.jointVel;
    jointAcc = params.jointAcc;
    
    % Number of joints
    n = size(jointPos, 1);
    
    % Link twists and accelerations
    V = zeros(6,n+1);
    Vdot = zeros(6,n+1);
    Vdot(:,1) = [0 0 0 -g];
    
    % Screw axes of each joint, expressed in the local link frame
    Ai = zeros(6,n);
    T = zeros(4,4,n);
    Mi = eye(4);
    
    % Initialize AdTi,i-1 to reuse it later
    AdTi = zeros(6,6,n+1);
    AdTi(:,:,n+1) = adjoint(pinv(M(:,:,n+1))); % This adjoint transformation takes Ftip and transforms it into the n-1 link frame (fixed transformation).
    
    for ii=1:n
        Mi = Mi * M(:,:,ii);
        Ai(:,ii) = adjoint(pinv(Mi)) * S(:,ii);
        
        % Eq. (8.50)
        T(:,:,ii) = twist2ht(Ai(:,ii), -jointPos(ii)) * pinv(M(:,:,ii)); % Alternative: T01 - FK in Body Frame: fkine(A(:,1), M01, theta(1))
        
        AdTi(:,:,ii) = adjoint(T(:,:,ii));
        
        % Eq. (8.51): Link velocities
        V(:,ii+1) = AdTi(:,:,ii) * V(:,ii) ... 
            + Ai(:,ii) * jointVel(ii);
        
        % Eq. (8.52): Link accelerations
        Vdot(:,ii+1) = AdTi(:,:,ii) * Vdot(:,ii) ... 
            + ad(V(:,ii+1)) * Ai(:,ii) * jointVel(ii) ...
            + Ai(:,ii) * jointAcc(ii);
    end
    
    % Backward iterations
    tau = zeros(n,1);
    Fi = Ftip;
    
    for ii = n : -1 : 1
        % Eq. (8.53): Equation of motion of a single rigid body
        Fi = AdTi(:,:,ii+1)' * Fi ...
            + G(:,:,ii) * Vdot(:,ii+1) ...
            - ad(V(:,ii+1))' * (G(:,:,ii) * V(:,ii+1));
        
        % Eq. (8.54): Joint torques
        tau(ii) = Fi' * Ai(:,ii);  
    end
end