% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = true;
sim_params.showFrames = false;
sim_params.logStep = 1;
sim_params.log_data = true;

% Time step
sim_params.dt = 1e-3;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time
sim_params.totalTime = 62;


% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 1;

%% Input text file 
inputFileName = 'experiments/knot/input.txt';

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

%% Input parameters
% geometry parameters
geom.shell_h = 0;
geom.rod_r0 = 0.0016; % for contact
% material parameters
material.density = 1180;
material.youngs_rod = 0.18e6; % not used
material.youngs_shell = 0;
material.poisson_rod = 0.5;
material.poisson_shell = 0;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
% env.ext_force_list = ["selfContact", "selfFriction", "viscous"];
env.ext_force_list = ["selfContact", "viscous"];
% env.ext_force_list = ["empty"];

material.contact_stiffness = 1;
% % material.mu = 0.3;
% % material.velTol = 1e-4;

% environment parameters
% env.g = [0, 0, -9.81]';
env.eta = 0.1;

%% Tolerance on force function. 

sim_params.tol = 1e-3;
sim_params.ftol = 1e-8;
sim_params.dtol = 1e-2;

%% Boundary conditions
start_nodes = [1, 2];
end_nodes = [size(nodes,1), size(nodes,1) - 1];
fixed_node_indices = [start_nodes, end_nodes];
fixed_edge_indices = [];

%% logging
input_log_node = 150;

%% Plot dimensions
sim_params.plot_x = [-0.5,0.5];
sim_params.plot_y = [-0.5,0.5];
sim_params.plot_z = [-0.5,0.5];

%% changing boundary conditions
pull_speed = 0.006;  % m/s
wait_time = 2;  % seconds
release_time = 0;  % seconds
pull_time = 60;  % seconds