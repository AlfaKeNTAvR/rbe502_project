clear, clc, close all
addpath('kinematics');
addpath('kinematics/lib');

plotOn = false;

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the robot and display it in the home configuration
robot = make_robot();
robot.plot(zeros(1,3));

% Create a kinematic model of the robot
[S,M] = make_kinematics_model();
n = size(S,2); % read the number of joints


%% Control the motion of the robot between 2 set points
fprintf('----------------------Kinematic Control of a 3-DoF Arm--------------------\n');

path = [0 0.3 0.15;
        0.25 0 0.1; ...
        0.25 0.25 0.2]';

nPts = size(path,2);

fprintf('Calculating the Inverse Kinematics... ');
robot.plot(zeros(1,3)); hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
% title('Inverse Dynamics Control');

% Calculate the inverse kinematics
waypoints = zeros(n,nPts);

% Initialize currentPose and currentQ.
currentPose = M(1:3,4);
currentQ = zeros(1,n);

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

    % Update joints solutions with the current solution.
    waypoints(:,ii) = currentQ;

end


fprintf('Done.\n');

% Now, for each pair of consecutive waypoints, we will calculate a
% trajectory between these two points.
fprintf('Generating the Trajectory... ');
nbytes = fprintf('0%%');

% Inititalize the variables where we will store the torque profiles, joint
% positions, and time, so that we can display them later.
jointPos = [];
jointVel = [];
jointAcc = [];
t = [];

for jj = 1 : nPts - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(jj/(nPts - 1)));
   
    % Initialize the time vector
    dt = 1e-3;       % time step [s]
    t  = 0 : dt : 5; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    jointPos_prescribed = zeros(n,size(t,2)); % Joint Variables (Prescribed)
    jointVel_prescribed = zeros(n,size(t,2)); % Joint Velocities (Prescribed)
    jointAcc_prescribed = zeros(n,size(t,2)); % Joint Accelerations (Prescribed)
    tau_prescribed      = zeros(n,size(t,2)); % Joint Torques

    jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)

    % For each joint
    for ii = 1 : n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.dt = dt;
        params_traj.q = [waypoints(ii,jj) waypoints(ii,jj+1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];

        traj = make_trajectory('quintic', params_traj);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        jointPos_prescribed(ii,:) = traj.q;
        jointVel_prescribed(ii,:) = traj.v;
        jointAcc_prescribed(ii,:) = traj.a;
    end

    jointPos = [jointPos jointPos_prescribed];
    jointVel = [jointVel jointVel_prescribed];
    jointAcc = [jointAcc jointAcc_prescribed];
    t = [t t+t(end)*(jj-1)];
end

fprintf('\nDone. Simulating the robot...');

%% Animate the robot
title('Point2point Trajectory');
robot.plot(jointPos(:,1:100:end)','trail',{'r', 'LineWidth', 2});
fprintf('Done.\n');


%% Display the Joint Torques
figure, hold on, grid on
plot(t, jointPos(1,:), 'Linewidth', 2);
plot(t, jointPos(2,:), 'Linewidth', 2);
plot(t, jointPos(3,:), 'Linewidth', 2);
title('Joint Position Profiles');
xlabel('Time [s]'), ylabel('Position [rad]');
legend({'Joint 1', 'Joint 2', 'Joint 3'});
set(gca, 'FontSize', 14);

figure, hold on, grid on
plot(t, jointVel(1,:), 'Linewidth', 2);
plot(t, jointVel(2,:), 'Linewidth', 2);
plot(t, jointVel(3,:), 'Linewidth', 2);
title('Joint Velocity Profiles');
xlabel('Time [s]'), ylabel('Velocity [rad/s]');
legend({'Joint 1', 'Joint 2', 'Joint 3'});
set(gca, 'FontSize', 14);

figure, hold on, grid on
plot(t, jointAcc(1,:), 'Linewidth', 2);
plot(t, jointAcc(2,:), 'Linewidth', 2);
plot(t, jointAcc(3,:), 'Linewidth', 2);
title('Joint Acceleration Profiles');
xlabel('Time [s]'), ylabel('Acceleration [rad/s^2]');
legend({'Joint 1', 'Joint 2', 'Joint 3'});
set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');