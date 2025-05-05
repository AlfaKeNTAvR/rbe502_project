clear, clc, close all

addpath('controllers');
addpath('dynamics');
addpath('kinematics');
addpath('kinematics/lib');

%% Simulation parameters:
controller = 'CTC'; % Choose between 'PD', 'PID', or 'CTC'
load = 1.0; % Load on the end-effector [kg]
animateRobot = true; % Set to true to animate the robot
saveMovie = false; % Set to true to save the animation
plotting = true; % Set to true to plot the results


global I; % Initialize the global variable I for integral state

%% Data logging for plotting
global actualEEPositions desiredEEPositions EEPositionErrors;
global desiredJointPositions jointPositionErrors;
global jointTorques;

actualEEPositions = [];
desiredEEPositions = [];
EEPositionErrors = [];
desiredJointPositions = [];
jointPositionErrors = [];
jointTorques = [];

I = zeros(3,1);
simulationTime = 0;

% Create the robot
robot = make_robot();

% Create a kinematic model of the robot
[S,M] = make_kinematics_model();
n = size(S,2); % read the number of joints


%% Control the motion of the robot between 2 set points
fprintf('----------------------Kinematic Control of a 3-DoF Arm--------------------\n');

path = [0.2 -0.15 0.2; ...
        0.2 -0.15 0.0; ...
        -0.2 -0.15 0.0; ...
        -0.2 -0.3 0.1; ...
        0.2 -0.3 0.1; ...
        0.2 -0.15 0.0; ...
        0.2 -0.15 0.2; ...
        ]';
nPts = size(path,2);
Time = 5;
% SimulationTines is an array of nPts-1 Time
simulationTimes = [Time * ones(1, nPts-1)]; % Simulation time for each segment

fprintf('Calculating the Inverse Kinematics... ');
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Calculate the inverse kinematics
waypoints = zeros(n,nPts);

% Initialize currentPose and currentQ.
currentPose = M(1:3,4);
currentQ = zeros(1,n);
trajectory = [];
times = [];

for ii = 1 : nPts
    targetPose = path(:, ii);
    errorPose = norm(targetPose - currentPose);

    while errorPose > 1e-5
        Ja = jacoba(S,M,currentQ);

        % Gradient descent method (Jacobian Transpose):
        alpha = 1;
        deltaQ = alpha * Ja' * (targetPose - currentPose); 

        % Update currentQ and errorPose.
        currentQ = currentQ + deltaQ';
        errorPose = norm(targetPose - currentPose);
        
        % Calculate the twist representing the robot's current pose.
        T = fkine(S,M,currentQ,'space');
        currentPose = T(1:3, 4);
    end

    q1 = currentQ(1);
    q2 = currentQ(2);
    q3 = currentQ(3);
    l1 = 0.3; % Length of Link 1 [m]
    l2 = 0.3; % Length of Link 2 [m]
    l3 = 0.15; % Length of Link 3 [m]

    rho_1 = l2 * cos(q2);
    rho_2 = l3 * cos(q2 + q3);

    Point_1 = [0; 0; l1;];
    Point_2 = [rho_1 * cos(q1);
                rho_1 * sin(q1);
                l1 + l2 * sin(q2)
                ];
    Point_3 = [(rho_1 + rho_2) * cos(q1);
                (rho_1 + rho_2) * sin(q1);
                l1 + l2 * sin(q2) + l3 * sin(q2 + q3)
                ];

    % Update joints solutions with the current solution.
    waypoints(:,ii) = currentQ;

    if ii == 1
        previousQ = currentQ; % Store the first solution
    end

    %% Controllers:
    X0 = [previousQ(1), previousQ(2), previousQ(3), 0, 0, 0];  % Initial State
    q_des = [currentQ(1), currentQ(2), currentQ(3)]; % Desired State

    if ii > 1
        tf = simulationTimes(ii-1);  % Simulation time
        simulationTime = simulationTime + tf; % Update the total simulation time
        clear X  % Clear the variable 'X' to asvoid conflicts with the function

        %% PD Control:
        if strcmp(controller, 'PD')
            [T, X] = ode45(@(t, x) planarArmODE_PDGravity(t, x, q_des, load), [0 tf], X0);
        end
        

        %% PID Control:
        if strcmp(controller, 'PID')
            [T, X] = ode45(@(t, x) planarArmODE_PIDGravity(t, x, q_des, load), [0 tf], X0);
        end

        %% CTC and Trajectory Generation:
        if strcmp(controller, 'CTC')
            % Inititalize the variables where we will store the torque profiles, joint
            % positions, and time, so that we can display them later.
            jointPos = [];
            jointVel = [];
            jointAcc = [];
            t = [];

            dt = 1e-3;
            t_local = 0:dt:5;
    
            % Store segment trajectory
            jointPos_prescribed = zeros(n, length(t_local));
            jointVel_prescribed = zeros(n, length(t_local));
            jointAcc_prescribed = zeros(n, length(t_local));
    
            for jj = 1 : n
                params_traj.t = [0 t_local(end)];
                params_traj.dt = dt;
                params_traj.q = [waypoints(jj, ii-1), waypoints(jj, ii)];
                params_traj.v = [0 0];
                params_traj.a = [0 0];
                
                traj = make_trajectory('quintic', params_traj);
                
                jointPos_prescribed(jj,:) = traj.q;
                jointVel_prescribed(jj,:) = traj.v;
                jointAcc_prescribed(jj,:) = traj.a;
            end

            % Simulate the segment.
            [T, X] = ode45(@(t, x) planarArmODE_CTCGravity(t, x, t_local, ...
                jointPos_prescribed, jointVel_prescribed, jointAcc_prescribed, load), ...
                [0 t_local(end)], X0);
        end

        trajectory = [trajectory; X]; % Store the trajectory
        % We need to add the previous last time to all new times to keep it running.
        % This is because the ode45 function returns the time from 0 to tf.
        % We need to add the previous time to all new times to keep it running.
        T = T + simulationTime; % Update the time to keep it running
        times =[times; T]; % Store the time
        previousQ = X(end, 1:3); % Update the previousQ for the next iteration
        I = zeros(3,1); % Reset the integral state for the next iteration
    end
end

fprintf('\nDone. Simulating the robot...');

%% Animate the robot
if animateRobot
    title(sprintf('3D glue application: %s', controller));

    % Start a timer:
    tic
    fps = 30;
    N = size(trajectory, 1);
    n_frames = fps * simulationTime * 0.478;

    % Downsample to fixed number of frames
    idx = round(linspace(1, N, n_frames));
    frames = trajectory(idx, 1:3);

    if saveMovie
        movie_name = sprintf('movies/glue/glue_application_%s.mp4', controller);
        robot.plot(frames, 'fps', fps, 'trail', {'r', 'LineWidth', 2}, 'view', [205, 30], 'workspace', [-0.75 0.75 -0.75 0.75 -0.05 0.5], 'movie', movie_name);
    else
        robot.plot(frames, 'fps', fps, 'trail', {'r', 'LineWidth', 2}, 'view', [205, 30], 'workspace', [-0.75 0.75 -0.75 0.75 -0.05 0.5]);
    end

    
    % End the timer:
    toc
    % Show the duration:
    fprintf('Animation time: %.2f seconds\n', toc);
    fprintf('\nDone.\n');
end


if plotting
    %% Plot Joint Positions
    N_desired = size(desiredJointPositions, 1);
    N_traj = size(trajectory, 1);               

    % Generate uniform time vectors
    t_des = linspace(0, 1, N_desired); % normalized time
    t_traj = linspace(0, 1, N_traj);   % normalized to same range

    % Interpolate each joint
    desiredJointPositions = interp1(t_des, desiredJointPositions, t_traj);

    
    figure('Name', 'Joint 1 Positions', 'Position', [100, 25, 400, 775]);
    subplot(3,1,1);
    plot(times, trajectory(:,1), 'b-', 'LineWidth', 1); hold on;
    plot(times, desiredJointPositions(:,1), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend('Actual', 'Desired');
    title('Joint 1 Positions');
    grid on;

    subplot(3,1,2);
    plot(times, trajectory(:,2), 'b-', 'LineWidth', 1); hold on;
    plot(times, desiredJointPositions(:,2), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend('Actual', 'Desired');
    title('Joint 2 Positions');
    grid on;

    subplot(3,1,3);
    plot(times, trajectory(:,3), 'b-', 'LineWidth', 1); hold on;
    plot(times, desiredJointPositions(:,3), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend('Actual', 'Desired');
    title('Joint 3 Positions');
    grid on;

    %% Plot Joint Positions Error
    N_desired = size(jointPositionErrors, 1);
    N_traj = size(trajectory, 1);               

    % Generate uniform time vectors
    t_des = linspace(0, 1, N_desired); % normalized time
    t_traj = linspace(0, 1, N_traj);   % normalized to same range

    % Interpolate each joint
    jointPositionErrors = interp1(t_des, jointPositionErrors, t_traj);

    figure('Name', 'Joint Position Errors', 'Position', [100, 25, 400, 775]);
    subplot(3,1,1);
    plot(times, jointPositionErrors(:,1), 'b-', 'LineWidth', 1) ; hold on;
    plot(times, ones(size(times, 1)) * 0.05, 'r--', 'LineWidth', 1);
    plot(times, ones(size(times, 1)) * -0.05, 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Error (rad)');
    legend('Error', '+5%', '-5%');
    title('Joint 1 Position Error');
    grid on;

    subplot(3,1,2);
    plot(times, jointPositionErrors(:,2), 'r-', 'LineWidth', 1); hold on;
    plot(times, ones(size(times, 1)) * 0.05, 'k--', 'LineWidth', 1);
    plot(times, ones(size(times, 1)) * -0.05, 'k--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Error (rad)');
    legend('Error', '+5%', '-5%');
    xlabel('Time (s)');
    ylabel('Error (rad)');
    title('Joint 2 Position Error');
    grid on;

    subplot(3,1,3);
    plot(times, jointPositionErrors(:,3), 'k-', 'LineWidth', 1); hold on;
    plot(times, ones(size(times, 1)) * 0.05, 'b--', 'LineWidth', 1);
    plot(times, ones(size(times, 1)) * -0.05, 'b--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Error (rad)');
    legend('Error', '+5%', '-5%');
    xlabel('Time (s)');
    ylabel('Error (rad)');
    title('Joint 3 Position Error');
    grid on;


    %% Plot Joint Velocities
    figure('Name', 'Joint Velocities', 'Position', [100, 25, 400, 775]);
    subplot(3,1,1);
    plot(times, trajectory(:,4), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    title('Joint 1 Velocity');
    grid on;

    subplot(3,1,2);
    plot(times, trajectory(:,5), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    title('Joint 2 Velocity');
    grid on;

    subplot(3,1,3);
    plot(times, trajectory(:,6), 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    title('Joint 3 Velocity');
    grid on;

    %% Plot Joint Torques
    N_desired = size(jointTorques, 1);
    N_traj = size(trajectory, 1);   

    % Generate uniform time vectors
    t_des = linspace(0, 1, N_desired); % normalized time
    t_traj = linspace(0, 1, N_traj);   % normalized to same range

    % Interpolate each joint
    jointTorques = interp1(t_des, jointTorques, t_traj);

    figure('Name', 'Joint Torques', 'Position', [100, 25, 400, 775]);
    % Joint 1 Torque
    subplot(3,1,1);
    plot(times, jointTorques(:,1), 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Joint 1 Torque');
    grid on;

    % Joint 2 Torque
    subplot(3,1,2);
    plot(times, jointTorques(:,2), 'r-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Joint 2 Torque');
    grid on;

    % Joint 3 Torque
    subplot(3,1,3);
    plot(times, jointTorques(:,3), 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Joint 3 Torque');
    grid on;
end

fprintf('Program completed successfully.\n');