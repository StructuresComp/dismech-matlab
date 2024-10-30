clc
clear all
% add to path 
addpath util_functions/
addpath contact_functions/
addpath rod_dynamics/
addpath shell_dynamics/
addpath external_forces/

%% input
robotDescription

% create geometry
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
twist_angles=zeros(n_edges_rod_only+n_edges_rod_shell,1);

%% Create the soft robot structure
MultiRod = MultiRod(geom, material, twist_angles,...
    nodes, edges, rod_nodes, shell_nodes, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, ...
    face_nodes, sign_faces, face_edges, sim_params);

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
                MultiRod.refLen(s), elStretchRod(s,:),MultiRod);

        else % shell
            stretch_springs (s) = stretchSpring (...
                MultiRod.refLen(s), ...
                elStretchShell(s-(size(elStretchRod,1)),:), ...
                MultiRod, MultiRod.ks(s));
        end
    end
end

% bending and twisting spring
if(n_bend_twist==0)
    bend_twist_springs = [];
else
    for b=1:n_bend_twist
        bend_twist_springs(b) = bendTwistSpring ( ...
            elBendRod(b,:), elBendSign(b,:), MultiRod.voronoiRefLen(elBendRod(b,3)), [0 0], 0, MultiRod);
    end
end

% hinge bending spring
if(n_hinge==0)
    hinge_springs = [];
else
    if(~sim_params.use_midedge)
        for h=1:n_hinge
            hinge_springs(h) = hingeSpring (...
                0, elBendShell(h,:), MultiRod, MultiRod.kb);
        end
        hinge_springs = setThetaBar(hinge_springs, MultiRod);
    else
        hinge_springs = [];
    end
end

%% Prepare system
% Reference frame (Space parallel transport at t=0)
MultiRod = computeSpaceParallel(MultiRod);

% Material frame from reference frame and twist angle 
theta = MultiRod.q0(3*MultiRod.n_nodes+1:3*MultiRod.n_nodes+MultiRod.n_edges_dof); % twist angle
[MultiRod.m1, MultiRod.m2] = computeMaterialDirectors(MultiRod.a1,MultiRod.a2,theta); 

% Set rod natural curvature
bend_twist_springs = setkappa(MultiRod, bend_twist_springs);

% Reference twist
undef_refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, MultiRod.a1, MultiRod.tangent, ...
     zeros(n_bend_twist,1));
refTwist = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, MultiRod.a1, MultiRod.tangent, ...
     zeros(n_bend_twist,1));
MultiRod.refTwist = refTwist;
MultiRod.undef_refTwist = undef_refTwist;

%% Boundary Conditions

MultiRod.fixed_nodes = fixed_node_indices;
MultiRod.fixed_edges = fixed_edge_indices;
[MultiRod.fixedDOF, MultiRod.freeDOF] = FindFixedFreeDOF(MultiRod.fixed_nodes, MultiRod.fixed_edges, MultiRod.n_DOF, MultiRod.n_nodes);

% Visualize initial configuration and the fixed and free nodes: free nodes - blue, fixed - red
plot_MultiRod(MultiRod, 0.0, sim_params);

%% initial conditions on velocity / angular velocity (if any)


%% Time stepping scheme

Nsteps = round(sim_params.totalTime/sim_params.dt);
ctime = 0; % current time
current_pos = zeros(Nsteps,1);
time_arr = linspace(0,sim_params.totalTime,Nsteps);

log_node = input_log_node;
current_pos(1) = MultiRod.q0(3*log_node);

for timeStep = 1:Nsteps
    %% Precomputation at each timeStep: midedge normal shell bending
    if(sim_params.use_midedge)
        tau_0 = updatePreComp_without_sign(MultiRod.q, MultiRod);
    else
        tau_0 = [];
    end

    %%  DER error iteration
    [MultiRod, stretch_springs, bend_twist_springs, hinge_springs] = ...
        DERfun(MultiRod, stretch_springs, bend_twist_springs, hinge_springs, tau_0,environment,imc, sim_params);

    ctime = ctime + sim_params.dt
    
    % Update q
    MultiRod.q0 = MultiRod.q;

    % log values
    current_pos(timeStep) = MultiRod.q0(3*log_node);
    
    %% Saving the coordinates and Plotting
    
    if mod(timeStep, sim_params.plotStep) == 0
    % visualisation
        plot_MultiRod(MultiRod, ctime, sim_params);
    end
end

figure()
plot(time_arr,current_pos)
title('position of the free node with time')
xlabel('time [s]')
ylabel('position [m]')