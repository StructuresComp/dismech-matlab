% input: robotDescriptionParachute.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false;
sim_params.use_lineSearch = false;
sim_params.showFrames = false;
sim_params.log_data = true;
sim_params.logStep = 1;

% Time step
sim_params.dt = 1e-2;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time (it exits after t=totalTime)
sim_params.totalTime = 3; % 3 % sec

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 10;

%% Input parameters
% geometry parameters
geom.rod_r0 = 1e-3;
geom.shell_h = 1e-3;

% material parameters
material.density = 1500;
material.youngs_rod = 10e6;
material.youngs_shell = 10e8;
material.poisson_rod = 0.5;
material.poisson_shell = 0.3;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity", "aerodynamic"]; 

% environment parameters
env.g = [0, 0, -9.81]';
env.rho = 1;
env.Cd = 10;

%% Input text file 

% inputFileName = 'experiments/parachute/triangle_parachute_n4_python.txt';
% inputFileName = 'experiments/parachute/simplest_parachute.txt';
inputFileName = 'experiments/parachute/hexparachute_n6_python.txt';
% inputFileName = 'experiments/parachute/Copy_of_hexparachute_n6_python.txt';

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = [];
fixed_edge_indices = [];

input_log_node = 1;

%% Plot dimensions
sim_params.plot_x = [-2,2];
sim_params.plot_y = [-2,2];
sim_params.plot_z = [-20,0];