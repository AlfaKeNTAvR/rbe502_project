syms m1 m2 m3 ... Mass of the links
l1 l2 l3 ... Length of Links
q1 q2 q3 ... Elbow angles
q1_d q2_d q3_d ... Velocities
q1_dd q2_dd q3_dd ... Accelerations
g t real; % gravity and time

%% Parameters:
m1 = 0.5; % Mass of Link 1 [kg]
m2 = 0.5; % Mass of Link 2 [kg]
% m3 = 0.25; % Mass of Link 3 [kg]

l1 = 0.3; % Length of Link 1 [m]
l2 = 0.3; % Length of Link 2 [m]
l3 = 0.15; % Length of Link 3 [m]

g = 9.81; % Gravity [m/s^2]


%% Diagonal distances in XY plane:Tau
rho_1 = l2 * cos(q2);
rho_2 = l3 * cos(q2 + q3);


%% Points matrix:
Point_1 = [0; 0; l1;];
Point_2 = [rho_1 * cos(q1);
            rho_1 * sin(q1);
            l1 + l2 * sin(q2)
            ];
Point_3 = [(rho_1 + rho_2) * cos(q1);
            (rho_1 + rho_2) * sin(q1);
            l1 + l2 * sin(q2) + l3 * sin(q2 + q3)
            ];


%% State variables:
q = [q1; q2; q3;];
q_d = [q1_d; q2_d; q3_d;];
q_dd = [q1_dd; q2_dd; q3_dd;];


%% Velocities:
Vel_1 = compute_velocity(Point_1, q, q_d);
Vel_2 = compute_velocity(Point_2, q, q_d);
Vel_3 = compute_velocity(Point_3, q, q_d);


%% Energies:
Point_1_Energy = [0.5*m1*(Vel_1'*Vel_1); m1*g*Point_1(3);];
Point_2_Energy = [0.5*m2*(Vel_2'*Vel_2); m2*g*Point_2(3);];
Point_3_Energy = [0.5*m3*(Vel_3'*Vel_3); m3*g*Point_3(3);];
System_Kin = simplify(Point_1_Energy(1)) + simplify(Point_2_Energy(1)) + simplify(Point_3_Energy(1));
System_Pot = simplify(Point_1_Energy(2)) + simplify(Point_2_Energy(2)) + simplify(Point_3_Energy(2));


%% Lagrangian:
L = simplify(System_Kin) - simplify(System_Pot);
L_d_by_q = derivative(L,q);
L_d_by_q_d = derivative(L,q_d);
L_d_by_q_d_by_dt = compute_velocity(L_d_by_q_d,q_d,q_dd);
Tau = L_d_by_q_d_by_dt - L_d_by_q';
Tau = simplify(expand(Tau));


%% Compact form:
M = simplify(jacobian(Tau, q_dd));
G = simplify(subs(Tau, [q_dd; q_d], zeros(6,1)));
C = simplify(Tau - M*q_dd - G);


%% Helper functions:
% Derivative function:
function dldq = derivative(l,q)
    for i = 1:length(q)
        dldq(:,i) = diff(l,q(i));
    end
    dldq = simplify(dldq);
end

% Compute_velocity:
function vel = compute_velocity(p,q,dq)
    for i = 1:length(q)
        dpdq(:,i) = diff(p, q(i)) * dq(i);
    end
    vel = simplify(expand(sum(dpdq,2)));
end

