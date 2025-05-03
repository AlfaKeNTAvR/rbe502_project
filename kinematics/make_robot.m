function robot = make_robot()

%% Create the manipulator
L1 = 0.3;  % Lenght of Link 1 [m]
L2 = 0.3;  % Lenght of Link 2 [m]
L3 = 0.15; % Lenght of Link 3 [m]

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', pi/2, 'offset', 0), ...
                    Revolute('a', L2, 'd', 0, 'alpha', 0, 'offset', 0), ...
                    Revolute('a', L3, 'd', 0, 'alpha', -pi/2, 'offset', 0)], ...
                    'name', 'RRR Manipulator');
end

