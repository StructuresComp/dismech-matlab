% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = true;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.showFrames = false;
sim_params.logStep = 1;
sim_params.log_data = true;
sim_params.bergou_DER = 0;
sim_params.FDM = 0;

% Time step
sim_params.dt = 1e-4;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time
if(sim_params.static_sim)
%     sim_params.totalTime = sim_params.dt;
    sim_params.totalTime = sim_params.dt*10;
else
    sim_params.totalTime = 1; % sec
end

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 100;

%% Input text file 
% inputFileName = 'experiments/rodCantilever/horizontal_rod_n3.txt';
inputFileName = 'experiments/rodCantilever/horizontal_rod_n21.txt';
% inputFileName = 'experiments/rodCantilever/horizontal_rod_n51.txt';
% inputFileName = 'experiments/rodCantilever/horizontal_rod_n101.txt';

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

%% Input parameters
% geometry parameters
geom.shell_h = 0;
geom.rod_r0 = 0.001; % for contact
% % geom cross section of rod
b = 0.02;
h = 0.001;
geom.Axs = b*h;
geom.Ixs1 = b*h^3/12;
geom.Ixs2 = h*b^3/12;
geom.Jxs = b*h^3/6;

% material parameters
material.density = 1200;
material.youngs_rod = 2e6; % not used
material.youngs_shell = 0;
material.poisson_rod = 0.5;
material.poisson_shell = 0;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity"]; 

% environment parameters
env.g = [0, 0, -9.81]';

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = find(nodes(:,1)<=0.01)';
% fixed_node_indices = [];
fixed_edge_indices = [];
% fixed_node_indices = [1,2];
% fixed_edge_indices = [1];

%% logging
input_log_node = size(nodes,1);

%% Plot dimensions
sim_params.plot_x = [0,0.1];
sim_params.plot_y = [-0.05,0.05];
sim_params.plot_z = [-0.05,0.05];
