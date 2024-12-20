% input: robotDescription.m

sim_params.static_sim = false;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.floor_present = false;
sim_params.showFloor = false;
sim_params.logStep = 1;
sim_params.log_data = true;

% Time step
sim_params.dt = 1e-3;

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
sim_params.plotStep = 10;

%% Input text file 
% inputFileName = 'experiments/rodCantilever/horizontal_rod_n26.txt';
inputFileName = 'experiments/rodCantilever/horizontal_rod_n21.txt';
% inputFileName = 'experiments/rodCantilever/horizontal_rod_n101.txt';

% reading the input text file
[rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes] = inputProcessorNew(inputFileName);

%% Input parameters
% geometry parameters
geom.shell_h = 0;
geom.rod_r0 = 0.001;
% % geom cross section of rod
b = 0.02;
h = 0.001;
% geom.Axs = b*h;
% geom.Ixs = b*h^3/12;
% geom.Jxs = b*h^3/6;
geom.rod_r0 = h; % for contact

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

[environment,imc] = createEnvironmentAndIMCStructs(env,geom,material,sim_params);
%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = find(rod_nodes(:,1)<=0.01)';
fixed_node_indices = [fixed_node_indices, find(rod_nodes(:,1)>=0.09)'];
fixed_edge_indices = [];

for i=1:size(rod_edges,1)
    if ( ismember(rod_edges(i,1),fixed_node_indices) && ismember(rod_edges(i,2),fixed_node_indices) )
        fixed_edge_indices = [fixed_edge_indices, i];
    end
end

%% logging
input_log_node = size(rod_nodes,1);

%% Plot dimensions
sim_params.plot_x = [0,0.1];
sim_params.plot_y = [-0.05,0.05];
sim_params.plot_z = [-0.05,0.05];
