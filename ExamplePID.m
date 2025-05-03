clear, clc, close all

addpath('controllers');
addpath('dynamics');
addpath('kinematics');
addpath('kinematics/lib');

global I; % Initialize the global variable I for integral state
I = 0;


% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the robot
robot = make_robot();

% Create a kinematic model of the robot
[S,M] = make_kinematics_model();
n = size(S,2); % read the number of joints


%% Control the motion of the robot between 2 set points
fprintf('----------------------Kinematic Control of a 3-DoF Arm--------------------\n');

path = [0 0.3 0.0;
        0.25 0 0.1; ...
        0.25 0.25 0.2]';

nPts = size(path,2);

fprintf('Calculating the Inverse Kinematics... ');
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Calculate the inverse kinematics
waypoints = zeros(n,nPts);

% Initialize currentPose and currentQ.
currentPose = M(1:3,4);
currentQ = zeros(1,n);
trajectory = [];

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
    tf = 10;  % Simulation time
    clear X  % Clear the variable 'X' to asvoid conflicts with the function

    % PID Control:
    [T, X] = ode45(@(t, x) planarArmODE_PIDGravity(t, x, q_des), [0 tf], X0);

    trajectory = [trajectory; X]; % Store the trajectory
    previousQ = currentQ; 
end

fprintf('\nDone. Simulating the robot...');

%% Animate the robot
title('Point2point Trajectory');
% robot.plot(jointPos(:,1:100:end)','trail',{'r', 'LineWidth', 2});
% Start a timer:
tic
robot.plot(trajectory(1:280:end, 1:3),'trail',{'r', 'LineWidth', 2});
% End the timer:
toc
% Show the duration:
fprintf('Animation time: %.2f seconds\n', toc);
fprintf('\nDone.\n');


%% Display the Joint Torques
% figure, hold on, grid on
% plot(t, jointPos(1,:), 'Linewidth', 2);
% plot(t, jointPos(2,:), 'Linewidth', 2);
% plot(t, jointPos(3,:), 'Linewidth', 2);
% title('Joint Position Profiles');
% xlabel('Time [s]'), ylabel('Position [rad]');
% legend({'Joint 1', 'Joint 2', 'Joint 3'});
% set(gca, 'FontSize', 14);

% figure, hold on, grid on
% plot(t, jointVel(1,:), 'Linewidth', 2);
% plot(t, jointVel(2,:), 'Linewidth', 2);
% plot(t, jointVel(3,:), 'Linewidth', 2);
% title('Joint Velocity Profiles');
% xlabel('Time [s]'), ylabel('Velocity [rad/s]');
% legend({'Joint 1', 'Joint 2', 'Joint 3'});
% set(gca, 'FontSize', 14);

% figure, hold on, grid on
% plot(t, jointAcc(1,:), 'Linewidth', 2);
% plot(t, jointAcc(2,:), 'Linewidth', 2);
% plot(t, jointAcc(3,:), 'Linewidth', 2);
% title('Joint Acceleration Profiles');
% xlabel('Time [s]'), ylabel('Acceleration [rad/s^2]');
% legend({'Joint 1', 'Joint 2', 'Joint 3'});
% set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');