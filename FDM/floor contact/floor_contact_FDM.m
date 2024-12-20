% check analytical gradient against fdm

clear all;
close all;
clc;

addpath ../../util_functions/
fprintf('FDM verification of ground contact jacobian\n');


%% element: triangle

% q values
n_nodes = 2;
n_dof = 3*n_nodes;
q0 = rand(n_dof,1);

q = q0 + 0.1*rand(n_dof,1);

dt = 1e-2;

imc.floor_z = -0.5;
imc.k_c_floor = 100;
imc.h = 1e-3;
imc.delta_floor = 1e0;
imc.omega = 20; % # iters before jacobian for contact forces is used
imc.scale = 1/imc.h;
imc.floor_has_friction = false;

[F_floorContact, J_floorContact] = computeFloorContactAndFriction(imc, dt, q, q0, n_nodes, n_dof)


%% FDM check for gradient and hessian
change = 1e-8;

J_floor_contact_FDM = zeros(n_dof,n_dof);

for c = 1:n_dof
    q_change = q;
    q_change(c) = q(c) + change;

    % changes in the energy
    [F_floorContact_change] = computeFloorContactAndFriction(imc, dt, q_change, q0, n_nodes, n_dof);


    J_floor_contact_FDM(c,:) = (F_floorContact_change - F_floorContact) .* (1/change);

end

%%

h1 = figure();
h1.WindowState = 'maximized';
plot( reshape(J_floorContact,[n_dof*n_dof,1]), 'ro');
hold on
plot( reshape(J_floor_contact_FDM,[n_dof*n_dof,1]), 'b^');
hold off
legend('Analytical', 'Finite Difference');
xlabel('Index');
ylabel('Gradient');