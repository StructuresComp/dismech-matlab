% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = true; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.floor_present = false;

% Time step
sim_params.dt = 1e-2;

% gravity
gravity = 1; % or 0 for off

if (gravity==1)
    g = [0, 0, -9.81]';
else
    g = [0, 0, 0]';
end

sim_params.g = g;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time (it exits after t=totalTime)
sim_params.totalTime = 0.2; % sec

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 10;

%% Input parameters
% geometry parameters
geom.rod_r0 = 1e-3;
geom.shell_h = 1e-3;
geom.rod_cyclicity = 0;

% material parameters
material.density = 1200;
material.youngs_rod = 2e7; % not used
material.youngs_shell = 2e9;
material.poisson_rod = 0.5;
material.poisson_shell = 0.5;

% environment parameters
environment.mu = 0.25;
environment.eta = 0;
environment.Cd = 0;
environment.rho = 1;

% point force
environment.ptForce = [0, 0, 0];
environment.ptForce_node = 1;

% imc
imc.compute_friction = true;
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
imc.floor_has_friction = true;
imc.floor_z = -0.5;


%% Input text file 
% inputFileName = 'experiments/shellCantilever/input_shell_cantilever_less_dense.txt';
inputFileName = 'experiments/shellCantilever/input_shell_cantilever_most_dense.txt';
% % inputFileName = 'experiments/shellCantilever/input_shell_cantilever_uniform4.txt';

% reading the input text file
[rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes] = inputProcessorNew(inputFileName);

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = find(shell_nodes(:,1)<=0.01)';
fixed_edge_indices = [];

free_edge_nodes = find(shell_nodes(:,1)>=0.1)';
input_log_node = free_edge_nodes(1);

%% Plot dimensions
sim_params.plot_x = [0,0.1];
sim_params.plot_y = [-0.05,0.05];
sim_params.plot_z = [-0.05,0.05];
