% input: robotDescription.m

sim_params.static_sim = 0;
sim_params.TwoDsim = 1;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = true;
sim_params.showFrames = false;
sim_params.logStep = 1;
sim_params.log_data = true;
sim_params.bergou_DER = 0;
sim_params.FDM = 0;

% Time step
sim_params.dt = 1e-1;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 150;

% Total simulation time
if(sim_params.static_sim)
%     sim_params.totalTime = sim_params.dt;
    sim_params.totalTime = sim_params.dt*10;
else
    sim_params.totalTime = 10; % sec
end

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 1;

%% Input text file 
inputFileName = 'experiments/gripping/gripping_rod_n49.txt';

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

%% Input parameters
% geometry parameters
geom.shell_h = 0;
geom.rod_r0 = 0.001; % for contact

% material parameters
material.density = 1200;
material.youngs_rod = 2e6; % not used
material.youngs_shell = 0;
material.poisson_rod = 0.5;
material.poisson_shell = 0;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["floorContact", "floorFriction"]; 
% env.ext_force_list = ["selfContact", "floorContact"];
% env.ext_force_list = ["gravity"];

% environment parameters
env.g = [0,0,0]';
env.contact_stiffness = 20;
env.mu = 0.25;
env.floor_z = 0.05;
env.velTol = 1e-2;
material.contact_stiffness = 1e3;
material.mu = 0.25;

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = find(nodes(:,1)<=0.01)';
fixed_edge_indices = [];

%% logging
input_log_node = size(nodes,1);

%% Plot dimensions
sim_params.plot_x = [-0.5,0.5];
sim_params.plot_y = [-0.5,0.5];
sim_params.plot_z = [-0.5,0.5];
