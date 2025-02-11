% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = true;
sim_params.log_data = true;
sim_params.logStep = 10;
sim_params.showFrames = false;

% Time step
sim_params.dt = 1e-4;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 100;

% Total simulation time (it exits after t=totalTime)
sim_params.totalTime = 1; % sec

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 100;

%% Input text file 
inputFileName = 'experiments/rodContact/input_straight_inclined_smaller_n21.txt';

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);
%% Input parameters
% geometry parameters
geom.rod_r0 = 1e-3;
geom.shell_h = 1e-3;
% 
% material parameters
material.density = 1500;
material.youngs_rod = 2e9;
material.youngs_shell = 0;
material.poisson_rod = 0.3;
material.poisson_shell = 0.5;
% 
%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity", "floorContact", "floorFriction"]; 

% environment parameters
env.g = [0, 0, -9.81]';
env.contact_stiffness = 20;
env.mu = 0.25;
env.floor_z = -0.05;
env.velTol = 1e-2;
material.contact_stiffness = 100;
material.mu = 0.25;

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = [];
fixed_edge_indices = [];
input_log_node = 1;

%% initial conditions
% u_init = [2:3:size(rod_nodes,1)*3; 0.1*ones(1,size(rod_nodes,1))];

%% Plot dimensions
sim_params.plot_x = [-0.1,0.1];
sim_params.plot_y = [-0.1,0.1];
sim_params.plot_z = [-0.1,0.1];
sim_params.view = [0,90]; % x-y plane
