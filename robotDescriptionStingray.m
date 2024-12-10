% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge or hinge for shell bending
sim_params.use_lineSearch = true; % turn on linesearch to help with convergence (esp. in contact rich scenarios)
sim_params.floor_present = false;
sim_params.log_data = true;
sim_params.logStep = 1;
sim_params.showFloor = false;
sim_params.dt = 1e-3; % sec % Time step
sim_params.maximum_iter = 50; % Maximum number of iterations in Newton Solver
sim_params.totalTime = 5; % sec % Total simulation time (it exits after t=totalTime)
sim_params.plotStep = 100; % How often the plot should be saved? (Set plotStep to 1 to show each plot)

%% Input parameters
% geometry parameters
geom.rod_r0 = 0; % not using rod
geom.shell_h = 1e-3;

% material parameters
material.density = 1057;
material.youngs_rod = 0; % not used
material.youngs_shell = 6e8; % 6e8 GPa works
material.poisson_rod = 0; % not used
material.poisson_shell = 0.3;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity", "buoyancy", "aerodynamic"];  

% environment parameters
env.g = [0, 0, -9.81]';
env.rho = 1000;
env.Cd = 0.3;

%% more visually nice
% material.youngs_shell = 6e8; % softer
% env.Cd = 0.05; % lower drag

%% better motion
material.youngs_shell = 6e9; % stiffer
env.Cd = 0.5; % higher drag

%%
[environment,imc] = createEnvironmentAndIMCStructs(env,geom,material,sim_params);

%% Input text file 
% inputFileName = 'experiments/stingRay/curve_input/stingray_n4_python.txt'; % choose
% inputFileName = 'experiments/stingRay/curve_input/stingray_n8_python.txt'; % choose
% inputFileName = 'experiments/stingRay/curve_input/stingray_n10_python.txt'; % choose
% inputFileName = 'experiments/stingRay/curve_input/stingray_n20_python.txt'; % choose
inputFileName = 'experiments/stingRay/curve_input/stingray_n15_python.txt'; % choose

% reading the input text file
[rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes] = inputProcessorNew(inputFileName);

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = [];
fixed_edge_indices = [];

[~,input_log_node] = ismember([0,0.5,0],shell_nodes,"rows");

%% Plot dimensions
sim_params.plot_x = [-1,1];
sim_params.plot_y = [-1,1];
sim_params.plot_z = [-1,1];