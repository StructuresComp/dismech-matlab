% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = true;
sim_params.floor_present = true;
sim_params.log_data = true;
sim_params.logStep = 1;
sim_params.showFloor = true;

% Time step
sim_params.dt = 1e-3;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time (it exits after t=totalTime)
sim_params.totalTime = 1; % sec

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 10;

%% Input text file 
% inputFileName = 'experiments/rodContact/new_input_for_contact.txt';
inputFileName = 'experiments/rodContact/input_straight_inclined.txt';

% reading the input text file
[rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes] = inputProcessorNew(inputFileName);

%% Input parameters
% geometry parameters
geom.rod_r0 = 1e-3;
geom.shell_h = 1e-3;

% material parameters
material.density = 1500;
material.youngs_rod = 20e6;
material.youngs_shell = 2e9;
material.poisson_rod = 0.5;
material.poisson_shell = 0.5;

% environment parameters
environment.mu = 0.25;
environment.eta = 0;
environment.Cd = 0;
environment.rho = 0;
environment.ptForce = [0, 0, 0]; % point force
environment.ptForce_node = size(rod_nodes,1);
% gravity
gravity = 1; % or 0 for off
if (gravity==1)
    g = [0, 0, -9.81]'; % g = [0, 0, -0.981]';
else
    g = [0, 0, 0]';
end
environment.g = g;

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


%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity", "floorContact", "floorFriction", "selfContact", "selfFriction"]; 

% environment parameters
env.g = [0, 0, -9.81]';
env.contact_stiffness = 100;
env.mu = 0.25;
env.floor_z = -0.5;
env.velTol = 1e-2;
material.contact_stiffness = 100;
material.mu = 0.25;


[environment,imc] = createEnvironmentAndIMCStructs(env,geom,material,sim_params);

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = []; % [4,6]
fixed_edge_indices = [];
input_log_node = size(rod_nodes,1);

%% initial conditions
% u_init = [3:3:9; -1*ones(1,3)];

%% Plot dimensions
sim_params.plot_x = [-1,1];
sim_params.plot_y = [-1,1];
sim_params.plot_z = [-1,1];
sim_params.view = [0,90]; % x-y plane
