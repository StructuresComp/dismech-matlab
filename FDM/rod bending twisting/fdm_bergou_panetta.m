% check analytical gradient against fdm

% clc
clear all
% close all
% add to path
addpath util_functions/
addpath contact_functions/
addpath rod_dynamics/
addpath shell_dynamics/
addpath external_forces/
addpath logging/
addpath(genpath('experiments')); 

%%
% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.showFrames = true;
sim_params.logStep = 1;
sim_params.log_data = true;
sim_params.bergou_DER = 1;

% Time step
sim_params.dt = 1e-2;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 100;

% Total simulation time
if(sim_params.static_sim)
%     sim_params.totalTime = sim_params.dt;
    sim_params.totalTime = sim_params.dt*10;
else
    sim_params.totalTime = 1; % sec
end

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 1;

%% Input parameters
% geometry parameters
geom.shell_h = 0;
geom.rod_r0 = 0.001;

% material parameters
material.density = 1200;
material.youngs_rod = 2e6; % not used
material.youngs_shell = 0;
material.poisson_rod = 0.5;
material.poisson_shell = 0;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity"]; 

% environment parameters
env.g = [0, 0, 0]';

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-10;
sim_params.dtol = 1e-10;

%% Boundary conditions
fixed_node_indices = [];
fixed_edge_indices = [];

%% logging
input_log_node = 1;

%% Plot dimensions
sim_params.plot_x = [0,0.1];
sim_params.plot_y = [-0.05,0.05];
sim_params.plot_z = [-0.05,0.05];
%%

nodes = rand(3,3);
edges = [1,2; 2,3];
face_nodes =[];
% create geometry
[nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, face_shell_edges, ...
    elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
    = createGeometry(nodes, edges, face_nodes);

% intialize twist angles for rod-edges to 0: this should be changed if one
% wants to start with a non-zero intial twist
% twist_angles=zeros(size(rod_edges,1)+size(rod_shell_joint_total_edges,1),1);
twist_angles = rand(2,1);

% create environment and imc structs
[environment,imc] = createEnvironmentAndIMCStructs(env,geom,material,sim_params);


%% Create the soft robot structure
softRobot = MultiRod(geom, material, twist_angles,...
    nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, ...
    face_nodes, sign_faces, face_edges, face_shell_edges, sim_params, environment);

%% Creating stretching, bending, twisting and hinge springs

n_stretch = size(elStretchRod,1) + size(elStretchShell,1);
n_bend_twist = size(elBendRod,1);

% stretching spring
if(n_stretch==0)
    stretch_springs = [];
else
    for s=1:n_stretch
        if (s <= size(elStretchRod,1)) % rod
            stretch_springs (s) = stretchSpring (...
                softRobot.refLen(s), elStretchRod(s,:),softRobot);

        else % shell
            stretch_springs (s) = stretchSpring (...
                softRobot.refLen(s), ...
                elStretchShell(s-(size(elStretchRod,1)),:), ...
                softRobot, softRobot.ks(s));
        end
    end
end

% bending and twisting spring
if(n_bend_twist==0)
    bend_twist_springs = [];
else
    for b=1:n_bend_twist
        bend_twist_springs(b) = bendTwistSpring ( ...
            elBendRod(b,:), elBendSign(b,:), [0 0], 0, softRobot);
    end
end

% shell bending spring
n_hinge = size(elBendShell,1);
n_triangle = softRobot.n_faces;
if(n_triangle==0)
    hinge_springs = [];
    triangle_springs = [];
else
    if(~sim_params.use_midedge)
        triangle_springs = [];
        for h=1:n_hinge
            hinge_springs(h) = hingeSpring (...
                0, elBendShell(h,:), softRobot, softRobot.kb);
        end
        hinge_springs = setThetaBar(hinge_springs, softRobot);
    else
        hinge_springs = [];
        for t=1:n_triangle
            triangle_springs(t) = triangleSpring(softRobot.face_nodes_shell(t,:), softRobot.face_edges(t,:), softRobot.face_shell_edges(t,:), softRobot.sign_faces(t,:), softRobot);
        end
    end
end

%%
softRobot.tangent = computeTangent(softRobot, softRobot.q0); 
softRobot = computeSpaceParallel(softRobot);

% Material frame from reference frame and twist angle
theta = softRobot.q0(3*softRobot.n_nodes+1:3*softRobot.n_nodes+softRobot.n_edges_dof); % twist angle
[softRobot.m1, softRobot.m2] = computeMaterialDirectors(softRobot.a1,softRobot.a2,theta);

% Set rod natural curvature
bend_twist_springs = setkappa(softRobot, bend_twist_springs);

% Reference twist
softRobot.undef_refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
    zeros(n_bend_twist,1));
softRobot.refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
    softRobot.undef_refTwist);

%% Deformed configuration
softRobot.q = softRobot.q0 + 0.1*rand(softRobot.n_DOF,1);
q = softRobot.q;

 % Compute time parallel reference frame
    [a1, a2] = computeTimeParallel(softRobot, softRobot.a1, softRobot.q0, q);

    % Compute reference twist
    tangent = computeTangent(softRobot, softRobot.q);
    refTwist = computeRefTwist_bend_twist_spring(bend_twist_springs, a1, tangent, softRobot.refTwist);

    % Compute material frame
    theta = q(3*softRobot.n_nodes + 1 : 3*softRobot.n_nodes + softRobot.n_edges_dof);
    [m1, m2] = computeMaterialDirectors(a1,a2,theta);

%%
[Fb, Jb, bend_twist_springs] = getFbJb(softRobot, bend_twist_springs, softRobot.q, m1, m2, sim_params);
[Ft, Jt, bend_twist_springs] = getFtJt(softRobot, bend_twist_springs, softRobot.q, refTwist, sim_params); % twisting

%% Trial
% [Fb_trial, Jb_trial, Ft_trial, Jt_trial, Jb_FDM_trial, Jt_FDM_trial, bend_twist_springs] = getFbJb_FtJt_and_FDM(softRobot, bend_twist_springs, softRobot.q, m1, m2, refTwist, sim_params);


change = 1e-8;
Jb_FDM = zeros(11,11);
Jt_FDM = zeros(11,11);

for c = 1:softRobot.n_DOF
    q_change = q;
    q_change(c) = q(c) + change;

    % Compute time parallel reference frame
    [a1, a2] = computeTimeParallel(softRobot, a1, q, q_change); % q or q0 in second last argument

    % Compute reference twist
    tangent = computeTangent(softRobot, q_change);
    refTwist_change = computeRefTwist_bend_twist_spring(bend_twist_springs, a1, tangent, refTwist);

    % Compute material frame
    theta = q_change(3*softRobot.n_nodes + 1 : 3*softRobot.n_nodes + softRobot.n_edges_dof);
    [m1, m2] = computeMaterialDirectors(a1,a2,theta);

    % changes in the gradient
    Fb_change = getFb(softRobot, bend_twist_springs, q_change, m1, m2);
    Jb_FDM(c,:) = (Fb_change - Fb) .* (1/change);

    Ft_change = getFt(softRobot, bend_twist_springs, q_change, refTwist_change); % twisting
    Jt_FDM(c,:) = (Ft_change - Ft) .* (1/change);

end

% Jb_FDM = Jb_FDM'; % why this?
% Jt_FDM = Jt_FDM'; % why this?
% diff_grad = gradE_FDM - gradE
% diff_hess = hessE_FDM - hessE
% 
% %%
% 

%%

figure(10);
subplot(2,1,1)
plot( reshape(Jb, [121,1]), 'ro');
hold on
plot( reshape(Jb_FDM, [121,1]), 'b^');
hold off
legend('Analytical', 'Finite Difference');
xlabel('Index');
ylabel('Hessian');
title('Bending hessian')

subplot(2,1,2)
plot( reshape(Jt, [121,1]), 'ro');
hold on
plot( reshape(Jt_FDM, [121,1]), 'b^');
hold off
legend('Analytical', 'Finite Difference');
xlabel('Index');
ylabel('Hessian');
title('Twisting hessian')
%%
% figure(20);
% subplot(2,1,1)
% plot( reshape(Jb_trial, [121,1]), 'ro');
% hold on
% plot( reshape(Jb_FDM_trial, [121,1]), 'b^');
% hold off
% legend('Analytical', 'Finite Difference');
% xlabel('Index');
% ylabel('Hessian');
% title('Bending hessian')
% 
% subplot(2,1,2)
% plot( reshape(Jt_trial, [121,1]), 'ro');
% hold on
% plot( reshape(Jt_FDM_trial, [121,1]), 'b^');
% hold off
% legend('Analytical', 'Finite Difference');
% xlabel('Index');
% ylabel('Hessian');
% title('Twisting hessian')