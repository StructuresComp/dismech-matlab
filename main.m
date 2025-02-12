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

% % Examples:
robotDescriptionRodCantilever
% robotDescriptionShellCantilever
% robotDescriptionParachute
% robotDescriptionRodContact
% robotDescriptionSquarePlate

% create geometry
[nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, ...
    elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
    = createGeometry(nodes, edges, face_nodes);

% intialize twist angles for rod-edges to 0: this should be changed if one
% wants to start with a non-zero intial twist
twist_angles=zeros(size(rod_edges,1)+size(rod_shell_joint_total_edges,1),1);

% create environment and imc structs
[environment,imc] = createEnvironmentAndIMCStructs(env,geom,material,sim_params);


%% Create the soft robot structure
softRobot = MultiRod(geom, material, twist_angles,...
    nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, ...
    face_nodes, sign_faces, face_edges, sim_params, environment);

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

% hinge bending spring
n_hinge = size(elBendShell,1);
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
softRobot.undef_refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
    zeros(n_bend_twist,1));
softRobot.refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
    softRobot.undef_refTwist);

%% Boundary Conditions

softRobot.fixed_nodes = fixed_node_indices;
for i=1:size(softRobot.Edges,1)
    if ( ismember(softRobot.Edges(i,1),fixed_node_indices) && ismember(softRobot.Edges(i,2),fixed_node_indices) )
        fixed_edge_indices = [fixed_edge_indices, i];
    end
end
if(sim_params.TwoDsim)
    fixed_edge_indices = [fixed_edge_indices, 1:softRobot.n_edges_dof]; % all rod thetas are fixed if it is a 2D sim
end
softRobot.fixed_edges = fixed_edge_indices;
[softRobot.fixedDOF, softRobot.freeDOF] = FindFixedFreeDOF(softRobot.fixed_nodes, softRobot.fixed_edges, softRobot.n_DOF, softRobot.n_nodes);

% Visualize initial configuration and the fixed and free nodes: free nodes - blue, fixed - red
plot_MultiRod(softRobot, 0.0, sim_params,environment,imc);

%% Initial conditions on velocity / angular velocity (if any)


%% Time stepping scheme

Nsteps = round(sim_params.totalTime/sim_params.dt);
ctime = 0; % current time
time_arr = linspace(0,sim_params.totalTime,Nsteps);

% containers for logging data
current_pos_x = zeros(Nsteps,1);
current_pos_y = zeros(Nsteps,1);
current_pos_z = zeros(Nsteps,1);
log_node = input_log_node;
current_pos_x(1) = softRobot.q0(3*log_node-2);
current_pos_y(1) = softRobot.q0(3*log_node-1);
current_pos_z(1) = softRobot.q0(3*log_node);

dof_with_time = zeros(softRobot.n_DOF+1,Nsteps);
dof_with_time(1,:) = time_arr;

for timeStep = 1:Nsteps
    if(sim_params.static_sim)
        environment.g = timeStep*environment.static_g/Nsteps; % ramp gravity
    end
    %% Precomputation at each timeStep: midedge normal shell bending
    if(sim_params.use_midedge)
        tau_0 = updatePreComp_without_sign(softRobot.q, softRobot);
    else
        tau_0 = [];
    end

    %%  Implicit stepping error iteration
    [softRobot, stretch_springs, bend_twist_springs, hinge_springs] = ...
        timeStepper(softRobot, stretch_springs, bend_twist_springs, hinge_springs, tau_0,environment,imc, sim_params);

    ctime = ctime + sim_params.dt

    % Update q
    softRobot.q0 = softRobot.q;

    %% Logging and animation
    current_pos_x(timeStep) = softRobot.q0(3*log_node-2);
    current_pos_y(timeStep) = softRobot.q0(3*log_node-1);
    current_pos_z(timeStep) = softRobot.q0(3*log_node);

    if(sim_params.log_data)
        if mod(timeStep, sim_params.logStep) == 0
            dof_with_time(2:end,timeStep) =  softRobot.q;
        end
    end
    if mod(timeStep, sim_params.plotStep) == 0
        plot_MultiRod(softRobot, ctime, sim_params, environment, imc);
    end
end

%% Saving data
[rod_data,shell_data] = logDataForRendering(dof_with_time, softRobot, Nsteps, sim_params.static_sim);

filename = "node_trajectory.xls";
writematrix(time_arr', filename, Sheet=1,Range='A1');
writematrix(current_pos_x, filename, Sheet=1,Range='B1');
writematrix(current_pos_y, filename, Sheet=1,Range='C1');
writematrix(current_pos_z, filename, Sheet=1,Range='D1');

%% Plots
% time trajectory
figure()
plot(time_arr,current_pos_x, time_arr, current_pos_y, time_arr, current_pos_z);
title('time trajectory of the node')
legend(['x'; 'y'; 'z'])
xlabel('t [s]')
ylabel('z [m]')
% space trajectory
figure()
plot3(current_pos_x, current_pos_y, current_pos_z);
title('space trajectory of the node')
legend(['x'; 'y'; 'z'])
xlabel('t [s]')
ylabel('z [m]')
