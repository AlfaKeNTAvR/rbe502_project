function q = ikin(S, M, currentQ, targetPose)
    for i=1:2

        [successLM, currentQLM] = LM(S, M, currentQ, targetPose, 1500, 0.5);
    
        if successLM
            q = currentQLM;
            result = true;
            return;
    
        else
             [successGD, currentQGD] = GD(S, M, currentQLM, targetPose, 500, 0.001);
    
             if successGD
                q = currentQGD;
                result = true;
                return;
    
             else
                 q = currentQGD;
                 result = false;
             end
        end
    end
end


function [success, currentQ] = NR(S,M,currentQ,targetPose,maxIterations)

    % Convert initialQ to currentPose twist.
    T = fkine(S,M,currentQ);
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';
    
    % Initialize starting parameters.
    errorPose = norm(targetPose - currentPose);
    currentIteration = 1;

    % Main loop.
    while errorPose > 1e-6
        if currentIteration > maxIterations
            break    
        end
    
        J = jacob0(S,currentQ);
        deltaQ = pinv(J) * (targetPose - currentPose);

        % Update currentQ and errorPose.
        currentQ = clampJointAngles(currentQ + deltaQ');
        errorPose = norm(targetPose - currentPose);
        
        % Update currentPose.
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';

        currentIteration = currentIteration + 1;
    end

    if errorPose < 1e-6 && checkJointLimits(currentQ)
        success = true;
    else
        success = false;
    end
end


function [success, currentQ] = LM(S,M,currentQ,targetPose,maxIterations,lambda)

    % Convert initialQ to currentPose twist.
    T = fkine(S,M,currentQ);
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';
    
    % Initialize starting parameters.
    errorPose = norm(targetPose - currentPose);
    currentIteration = 1;

    % Main loop.
    while errorPose > 1e-6
        if currentIteration > maxIterations
            break    
        end
    
        J = jacob0(S,currentQ);
        deltaQ = J' * pinv(J * J' + lambda^2 * eye(6)) * (targetPose - currentPose);

        % Update currentQ and errorPose.
        currentQ = clampJointAngles(currentQ + deltaQ');
        errorPose = norm(targetPose - currentPose);
        
        % Update currentPose.
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';

        currentIteration = currentIteration + 1;
    end

    if errorPose < 1e-6 && checkJointLimits(currentQ)
        success = true;
    else
        success = false;
    end
end


function [success, currentQ] = GD(S,M,currentQ,targetPose,maxIterations,alpha)

    % Convert initialQ to currentPose twist.
    T = fkine(S,M,currentQ);
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';
    
    % Initialize starting parameters.
    errorPose = norm(targetPose - currentPose);
    currentIteration = 1;

    % Main loop.
    while errorPose > 1e-6 
        if currentIteration > maxIterations
            break    
        end
    
        J = jacob0(S,currentQ);
        deltaQ = alpha * J' * (targetPose - currentPose); 

        % Update currentQ and errorPose.
        currentQ = clampJointAngles(currentQ + deltaQ');
        errorPose = norm(targetPose - currentPose);
        
        % Update currentPose.
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';

        currentIteration = currentIteration + 1;
    end

    if errorPose < 1e-6 && checkJointLimits(currentQ)
        success = true;
    else
        success = false;
    end
end


function clampedQ = clampJointAngles(currentQ)
    % Define joint limits
    qlim = [deg2rad(-180.0)  deg2rad(180.0);  % q(1)
            deg2rad(-125.0)  deg2rad(125.0);  % q(2)
            deg2rad(-138.0)  deg2rad(138.0);  % q(3)
            deg2rad(-270.0)  deg2rad(270.0);  % q(4)
            deg2rad(-133.5)  deg2rad(133.5);  % q(5)
            deg2rad(-270.0)  deg2rad(270.0)]; % q(6)

    
    % Clamp each element of currentQ to its corresponding limit
    clampedQ = max(min(currentQ, qlim(:,2)'), qlim(:,1)');
end


function allWithinLimits = checkJointLimits(currentQ)
    % Initialize the return variables
    isWithinLimits = zeros(size(currentQ)); % Array to hold per-joint within-limit status
    allWithinLimits = true; % Flag indicating if all joints are within limits

    qlim = [deg2rad(-180.0)  deg2rad(180.0);  % q(1)
        deg2rad(-125.0)  deg2rad(125.0);  % q(2)
        deg2rad(-138.0)  deg2rad(138.0);  % q(3)
        deg2rad(-270.0)  deg2rad(270.0);  % q(4)
        deg2rad(-133.5)  deg2rad(133.5);  % q(5)
        deg2rad(-270.0)  deg2rad(270.0)]; % q(6)
    
    % Loop through each joint angle
    for i = 1:length(currentQ)
        % Check if the current joint angle is within its limits
        if currentQ(i) >= qlim(i, 1) && currentQ(i) <= qlim(i, 2)
            isWithinLimits(i) = 1; % This joint is within limits
        else
            isWithinLimits(i) = 0; % This joint is not within limits
            allWithinLimits = false; % If any joint is out of limits, set the flag to false
        end
    end
end

