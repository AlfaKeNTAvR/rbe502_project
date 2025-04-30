clear; clc; close all;

%% Desired and Initial Joint Values
q1_0 = 0;
q2_0 = 0;
q3_0 = 0;

X0 = [q1_0, q2_0, q3_0, 0, 0, 0];  % Initial State
tf = 15;  % Simulation time

[T, X] = ode45(@(t, x) planarArmODEGrComp(t, x), [0 tf], X0);

%% Plot Joint Positions
figure('Name', 'Joint Values, GComp');
plot(T, X(:,1), 'b-', 'LineWidth', 1);
hold on;
plot(T, X(:,2), 'r-', 'LineWidth', 1);
plot(T, X(:,3), 'k-', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Joint Values (rad or m)');
legend('q1', 'q2', 'q3');
title('Joint Values');
grid on;

%% Plot Joint Velocities
figure('Name', 'Joint Velocities, GComp');
plot(T, X(:,4), 'b-', 'LineWidth', 1);
hold on;
plot(T, X(:,5), 'r-', 'LineWidth', 1);
plot(T, X(:,6), 'k-', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s or m/s)');
legend('q1_{dot}', 'q2_{dot}', 'q3_{dot}');
title('Joint Velocities');
grid on;