% input: robotDescription.m

sim_params.static_sim = true;
sim_params.TwoDsim = true;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.log_data = true;
sim_params.logStep = 1;
sim_params.showFrames = false;

% Time step
sim_params.dt = 1e-3;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time (it exits after t=totalTime)
sim_params.totalTime = sim_params.dt; % sec

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 1;

%% Input text file 
inputFileName = 'experiments/pneunet/input_straight_horizontal_shorter.txt';

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

%% Input parameters
% geometry parameters
geom.rod_r0 = 1e-3;
geom.shell_h = 1e-3;

% material parameters
material.density = 1200;
material.youngs_rod = 2e10;
material.youngs_shell = 10e5;
material.poisson_rod = 0.5;
material.poisson_shell = 0.5;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity", "pointForce"]; 

% environment parameters
env.g = [0, 0, -9.81]';

env.ptForce = [0, 0, 0];
% env.ptForce = 100*[1.75, 0, 0.07]; % point force
env.ptForce_node = size(nodes,1);

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = [1,2];
fixed_edge_indices = [1];
input_log_node = size(nodes,1);

%% Plot dimensions
% sim_params.plot_x = [-1,1];
% sim_params.plot_y = [-1,1];
% sim_params.plot_z = [-1,1];
sim_params.plot_x = [-0.1,0.1];
sim_params.plot_y = [-0.1,0.1];
sim_params.plot_z = [-0.1,0.1];
