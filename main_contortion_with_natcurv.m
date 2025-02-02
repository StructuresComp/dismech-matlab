clc
clear all
close all
% add to path
addpath util_functions/
addpath contact_functions/
addpath rod_dynamics/
addpath shell_dynamics/
addpath external_forces/
addpath logging/


sim_params = struct();
imc = struct();
geom = struct();
material = struct();
environment = struct();

robotDescriptionStraightContortion

[nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, ...
    elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
    = createGeometry(nodes, rod_edges, rod_shell_joint_edges, face_nodes);

n_nodes = size(nodes,1);
n_edges = size(edges,1);
n_edges_rod_only = size(rod_edges,1);
n_edges_shell_only = size(shell_edges,1);
n_hinge = size(elBendShell,1);
n_edges_rod_shell = size(rod_shell_joint_edges,1);
n_edges_shell = size(shell_edges,1);
twist_angles=zeros(n_edges_rod_only+n_edges_rod_shell,1);

%% Create the soft robot structure
softRobot = MultiRod(geom, material, twist_angles,...
    nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, ...
    face_nodes, sign_faces, face_edges, sim_params, environment);

%% Creating stretching, bending, twisting springs

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

% hinge bending spring
if(n_hinge==0)
    hinge_springs = [];
else
    if(~sim_params.use_midedge)
        for h=1:n_hinge
            hinge_springs(h) = hingeSpring (...
                0, elBendShell(h,:), softRobot, softRobot.kb);
        end
        hinge_springs = setThetaBar(hinge_springs, softRobot);
    else
        hinge_springs = [];
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

%% Boundary Conditions

softRobot.fixed_nodes = fixed_node_indices;
if(sim_params.TwoDsim)
    fixed_edge_indices = [fixed_edge_indices, 1:softRobot.n_edges_dof]; % all rod thetas are fixed
end
softRobot.fixed_edges = fixed_edge_indices;
[softRobot.fixedDOF, softRobot.freeDOF] = FindFixedFreeDOF(softRobot.fixed_nodes, softRobot.fixed_edges, softRobot.n_DOF, softRobot.n_nodes);

% Visualize initial configuration and the fixed and free nodes: free nodes - blue, fixed - red
plot_MultiRod(softRobot, 0.0, sim_params);

%% actuation
kappa_bar = [-0.0628525320866699/2.*ones(n_bend_twist,1) , zeros(n_bend_twist,1)]; % quartercircle
% kappa_bar = [-0.0628525320866699.*ones(n_bend_twist,1) , zeros(n_bend_twist,1)]; % semicircle
% kappa_bar = [-0.0628525320866699*1.5.*ones(n_bend_twist,1) , zeros(n_bend_twist,1)]; % 3quartercircle

bend_twist_springs = actuatekappa(bend_twist_springs, kappa_bar);

%% initial conditions on velocity / angular velocity (if any)
u0 = 0.1;
w0 = 2;
%% Time stepping scheme

Nsteps = round(sim_params.totalTime/sim_params.dt);
ctime = 0; % current time
current_pos = zeros(Nsteps,1);
time_arr = linspace(0,sim_params.totalTime,Nsteps);

log_node = input_log_node;
current_pos(1) = softRobot.q0(3*log_node);

dof_with_time = zeros(softRobot.n_DOF+1,Nsteps);
dof_with_time(1,:) = time_arr;

for timeStep = 1:Nsteps
    if(sim_params.static_sim)
        environment.g = timeStep*environment.static_g/Nsteps; % ramp gravity
    end

    % contorting moving BC
    if(ctime<0.15)
        softRobot.q0(1:3:3*size(softRobot.fixed_nodes,2)/2) = softRobot.q0(1:3:3*size(softRobot.fixed_nodes,2)/2) + u0*sim_params.dt.*ones(size(1:3:3*size(softRobot.fixed_nodes,2)/2,1));
    else
        for i=1:size(softRobot.fixed_edges,2)/2
            softRobot.q0(mapEdgetoDOF(i,softRobot.n_nodes)) = softRobot.q0(mapEdgetoDOF(i,softRobot.n_nodes)) + w0*sim_params.dt;
%         softRobot.q0(mapEdgetoDOF(2,softRobot.n_nodes)) = softRobot.q0(mapEdgetoDOF(2,softRobot.n_nodes)) + w0*sim_params.dt;
        end
    end

    %% Precomputation at each timeStep: midedge normal shell bending
    if(sim_params.use_midedge)
        tau_0 = updatePreComp_without_sign(softRobot.q, softRobot);
    else
        tau_0 = [];
    end

    %%  DER error iteration
    [softRobot, stretch_springs, bend_twist_springs, hinge_springs] = ...
        DERfun(softRobot, stretch_springs, bend_twist_springs, hinge_springs, tau_0,environment,imc, sim_params);

    ctime = ctime + sim_params.dt

    % Update q
    softRobot.q0 = softRobot.q;

    % log values
    current_pos(timeStep) = softRobot.q0(3*log_node);

    if(sim_params.log_data)
        if mod(timeStep, sim_params.logStep) == 0
            dof_with_time(2:end,timeStep) =  softRobot.q;
        end
    end

    %% Saving the coordinates and Plotting

    if mod(timeStep, sim_params.plotStep) == 0
        % visualisation
        plot_MultiRod(softRobot, ctime, sim_params);
    end
end

% [rod_data,shell_data] = logDataForRendering(dof_with_time, MultiRod, Nsteps);

figure()
plot(time_arr,current_pos(1:end))
title('position of the free node with time')
xlabel('time [s]')
ylabel('position [m]')
