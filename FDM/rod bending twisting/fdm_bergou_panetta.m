% check analytical gradient against fdm

clear all;
close all;
clc;

addpath ../util_functions/
fprintf('FDM verification of midedge normal based shell bending\n');

stiff = 1;
nu = 0.5;

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.floor_present = false;
sim_params.showFloor = false;
sim_params.logStep = 1;
sim_params.log_data = true;

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
sim_params.plotStep = 10;

sim_params.tol = 1e-4;
sim_params.ftol = 1e-10;
sim_params.dtol = 1e-10;
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
% environment parameters
env.g = [0, 0, -9.81]';
env.ext_force_list = ["gravity"]; 

%% initial configuration (non-zero init curvature)
rod_nodes = rand(3,3);
twist_angles = rand(2,1);

rod_edges = [1,2; 2,3];

shell_nodes = [];
face_nodes = [];
rod_shell_joint_edges = [];


[nodes, edges, rod_nodes, shell_nodes, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, ...
    elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
    = createGeometry(rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes);
n_nodes = size(nodes,1);
n_edges = size(edges,1);
n_edges_rod_only = size(rod_edges,1);
n_edges_shell_only = size(shell_edges,1);
n_hinge = size(elBendShell,1);
n_edges_rod_shell = size(rod_shell_joint_edges,1);
n_edges_shell = size(shell_edges,1);
% twist_angles=zeros(n_edges_rod_only+n_edges_rod_shell,1);

softRobot = MultiRod(geom, material, twist_angles,...
    nodes, edges, rod_nodes, shell_nodes, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, ...
    face_nodes, sign_faces, face_edges, sim_params, env);

n_bend_twist = size(elBendRod,1);
if(n_bend_twist==0)
    bend_twist_springs = [];
else
    for b=1:n_bend_twist
        bend_twist_springs(b) = bendTwistSpring ( ...
            elBendRod(b,:), elBendSign(b,:), [0 0], 0, softRobot);
    end
end
%% Prepare system
% Reference frame (Space parallel transport at t=0)
softRobot = computeSpaceParallel(softRobot);

% Material frame from reference frame and twist angle
theta = softRobot.q0(3*softRobot.n_nodes+1:3*softRobot.n_nodes+softRobot.n_edges_dof); % twist angle
[softRobot.m1, softRobot.m2] = computeMaterialDirectors(softRobot.a1,softRobot.a2,theta);

% Set rod natural curvature
bend_twist_springs = setkappa(softRobot, bend_twist_springs);

% Reference twist
undef_refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
    zeros(n_bend_twist,1));
refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
    zeros(n_bend_twist,1));
softRobot.refTwist = refTwist;
softRobot.undef_refTwist = undef_refTwist;

figure()
hold on;
for i=1:size(edges,1)
    n1 = edges(i,1);
    n2 = edges(i,2);
    n1pos = rod_nodes(:,n1);
    n2pos = rod_nodes(:,n2);
   
    plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ko-');
end
hold off;

%% Deformed configuration
softRobot.q = rand(softRobot.n_DOF,1);
q = softRobot.q;

 % Compute time parallel reference frame
    [a1, a2] = computeTimeParallel(softRobot, softRobot.a1, softRobot.q0, q);

    % Compute reference twist
    tangent = computeTangent(softRobot, softRobot.q);
    refTwist = computeRefTwist_bend_twist_spring(bend_twist_springs, a1, tangent, softRobot.refTwist);

    % Compute material frame
    theta = q(3*n_nodes + 1 : 3*n_nodes + softRobot.n_edges_dof);
    [m1, m2] = computeMaterialDirectors(a1,a2,theta);

%%
[Fb, Jb, bend_twist_springs] = getFbJb(softRobot, bend_twist_springs, softRobot.q, m1, m2);


change = 1e-8;
Jb_FDM = zeros(11,11);

for c = 1:softRobot.n_DOF
    q_change = q;
    q_change(c) = q(c) + change;

    % Compute time parallel reference frame
    [a1, a2] = computeTimeParallel(softRobot, a1, q, q_change); % q or q0 in second last argument

    % Compute reference twist
    tangent = computeTangent(softRobot, q_change);
    refTwist = computeRefTwist_bend_twist_spring(bend_twist_springs, a1, tangent, refTwist);

    % Compute material frame
    theta = q_change(3*n_nodes + 1 : 3*n_nodes + softRobot.n_edges_dof);
    [m1, m2] = computeMaterialDirectors(a1,a2,theta);

    % changes in the gradient
    Fb_change = getFb(softRobot, bend_twist_springs, q_change, m1, m2);
    Jb_FDM(c,:) = (Fb_change - Fb) .* (1/change);

end

Jb_FDM = Jb_FDM'; % why this?

% diff_grad = gradE_FDM - gradE
% diff_hess = hessE_FDM - hessE
% 
% %%
% 
h1 = figure();
plot( reshape(Jb, [121,1]), 'ro');
hold on
plot( reshape(Jb_FDM, [121,1]), 'b^');
hold off
legend('Analytical', 'Finite Difference');
xlabel('Index');
ylabel('Hessian');