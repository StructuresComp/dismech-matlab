% input: robotDescription.m

sim_params.static_sim = true;
sim_params.TwoDsim = false;
sim_params.use_midedge = true; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.floor_present = false;
sim_params.showFloor = false;
sim_params.logStep = 1;
sim_params.log_data = true;

% Time step
sim_params.dt = 1e-2;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time
if(sim_params.static_sim)
%     sim_params.totalTime = sim_params.dt;
    sim_params.totalTime = sim_params.dt*10;
else
    sim_params.totalTime = 0.8; % sec
end

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 1;

%% Input parameters
% geometry parameters
geom.rod_r0 = 1e-3;
geom.shell_h = 1e-3;

% material parameters
material.density = 1200;
material.youngs_rod = 0; % not used
material.youngs_shell = 2e6;
material.poisson_rod = 0;
material.poisson_shell = 0.5;

% environment parameters
environment.mu = 0.25;
environment.eta = 0;
environment.Cd = 0;
environment.rho = 1;

% point force
environment.ptForce = [0, 0, 0];
environment.ptForce_node = 1;

% gravity
gravity = 1; % or 0 for off
if (gravity==1)
    g = [0, 0, -9.81]';
else
    g = [0, 0, 0]';
end
environment.g = g;

% imc
imc.compute_friction = false;
imc.k_c = 100;
imc.k_c_floor = 100;
imc.contact_len = 2*geom.rod_r0;
imc.delta = 0.01*imc.contact_len;
imc.delta_floor = 0.05;
imc.omega = 20;
imc.h = geom.rod_r0;
imc.scale = 1/imc.h;
imc.C = [];
imc.mu_k = environment.mu;
imc.velTol = 1e-2;
imc.floor_has_friction = false;
imc.floor_z = -0.5;


%% Input text file 
% inputFileName = 'experiments/shellCantilever/input_shell_cantilever_less_dense.txt';
% inputFileName = 'experiments/shellCantilever/equilateral_mesh_60.txt';
% inputFileName = 'experiments/shellCantilever/random_mesh_20.txt';
% inputFileName = 'experiments/shellCantilever/random_mesh_40.txt';


FileName = strcat(mesh_types(mesh_type), '_mesh_', num2str(mesh_dense_nos(mesh_dense)), '.txt');
inputFileName = strcat('experiments/shellCantilever/', FileName)

% reading the input text file
[rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes] = inputProcessorNew(inputFileName);

[nodes, edges, rod_nodes, shell_nodes, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, ...
    elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
    = createGeometry(rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes);

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = find(shell_nodes(:,1)<=0.01)';
fixed_edge_indices = [];

for i=1:size(edges,1)
    if ( ismember(edges(i,1),fixed_node_indices) && ismember(edges(i,2),fixed_node_indices) )
        fixed_edge_indices = [fixed_edge_indices, i];
    end
end

%% logging
% p = find(shell_nodes(:,1)==0.1)';
% input_log_node = p(1);

p = find(shell_nodes(:,1)==0.1)';
if (isempty(p))
    p = find(shell_nodes(1,:)>0.1);
end
Nodes_p = [shell_nodes(p,:)';p];
input_log_node = Nodes_p(4,find(Nodes_p(2,:) == 0));

if (isempty(input_log_node))
    input_log_node = Nodes_p(4,1);
end

%% Plot dimensions
sim_params.plot_x = [0,0.1];
sim_params.plot_y = [-0.05,0.05];
sim_params.plot_z = [-0.05,0.05];
