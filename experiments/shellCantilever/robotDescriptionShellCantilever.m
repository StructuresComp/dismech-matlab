% input: robotDescription.m

sim_params.static_sim = true;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.showFrames = false;
sim_params.logStep = 1;
sim_params.log_data = true;

% Time step
sim_params.dt = 1e-2;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time
if(sim_params.static_sim)
%     sim_params.totalTime = sim_params.dt;
    sim_params.totalTime = sim_params.dt*5;
else
    sim_params.totalTime = 0.8; % sec
end

% How often the plot should be shown? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 1;

%% Input parameters
% geometry parameters
geom.rod_r0 = 0;
geom.shell_h = 1e-3;

% material parameters
material.density = 1200;
material.youngs_rod = 0; % not used
material.youngs_shell = 2e8;
material.poisson_rod = 0;
material.poisson_shell = 0.5;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity"]; 

% environment parameters
env.g = [0, 0, -9.81]';

%% Input text file 
mesh_dense_nos = [20,25,30,35,40,45,50,55,60,65];
mesh_types = ["equilateral" , "random" , "right" , "eq_algn"]; % type of mesh

% choose
mesh_dense = 1;
mesh_type = 1;


FileName = strcat(mesh_types(mesh_type), '_mesh_', num2str(mesh_dense_nos(mesh_dense)), '.txt');
inputFileName = strcat('experiments/shellCantilever/', FileName);

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices = find(nodes(:,1)<=0.01)';
fixed_edge_indices = [];

%% logging

p = find(nodes(:,1)==0.1)';
if (isempty(p))
    p = find(nodes(1,:)>0.1);
end
Nodes_p = [nodes(p,:)';p];
input_log_node = Nodes_p(4,find(Nodes_p(2,:) == 0));

if (isempty(input_log_node))
    input_log_node = Nodes_p(4,1);
end

%% Plot dimensions
sim_params.plot_x = [0,0.15];
sim_params.plot_y = [-0.05,0.05];
sim_params.plot_z = [-0.05,0.05];
