% input: robotDescription.m

sim_params.static_sim = true;
sim_params.TwoDsim = false;
sim_params.use_midedge = false; % boolean var to decide on using midedge normal or 
% hinge model for shell bending
sim_params.use_lineSearch = false;
sim_params.floor_present = false;
sim_params.showFloor = false;
sim_params.logStep = 1;
sim_params.log_data = true;

% Time step
sim_params.dt = 1e-2;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 25;

% Total simulation time
if(sim_params.static_sim)
%     sim_params.totalTime = sim_params.dt;
    sim_params.totalTime = sim_params.dt*2;
else
    sim_params.totalTime = 0.8; % sec
end

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
sim_params.plotStep = 1;

%% Input parameters
% geometry parameters
geom.rod_r0 = 0;
geom.shell_h = 1e-3;

% material parameters
material.density = 7750;
material.youngs_rod = 0; % not used
material.youngs_shell = 200e9;
material.poisson_rod = 0;
material.poisson_shell = 0.3;

%% external force list ["selfContact", "selfFriction", "floorContact", "floorFriction", "gravity", "buoyancy", "viscous", "aerodynamic","pointForce"]
env.ext_force_list = ["gravity"]; 

% % environment parameters
% env.g = [0, 0, -9.81]';

[environment,imc] = createEnvironmentAndIMCStructs(env,geom,material,sim_params);

%% Input text file 
mesh_dense_nos = [4, 20,25,30,35,40,45,50,55,60,65];
mesh_types = ["equilateral" , "random" , "right" , "eq_algn"]; % type of mesh

% choose
mesh_dense = 4;
mesh_type = 2;


FileName = strcat(mesh_types(mesh_type), '_mesh_', num2str(mesh_dense_nos(mesh_dense)), '.txt');
inputFileName = strcat('experiments/squarePlate/', FileName);

% reading the input text file
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

% create geometry
[nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, ...
    elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
    = createGeometry(nodes, edges, face_nodes);
%% Tolerance on force function. 

sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-2;

%% Boundary conditions
fixed_node_indices_bound1 = find(nodes(:,1)==0)';
fixed_node_indices_bound2 = find(nodes(:,2)==0)';
fixed_node_indices_bound3 = find(nodes(:,1)==0.1)';
fixed_node_indices_bound4 = find(nodes(:,2)==0.1)';

fixed_node_indices = [fixed_node_indices_bound1, fixed_node_indices_bound2, fixed_node_indices_bound3, fixed_node_indices_bound4];

fixed_edge_indices = [];

for i=1:size(edges,1)
    if ( ismember(edges(i,1),fixed_node_indices) && ismember(edges(i,2),fixed_node_indices) )
        fixed_edge_indices = [fixed_edge_indices, i];
    end
end

%% logging

% center point
target = [0.05, 0.05, 0];

% Compute the Euclidean distances from the target point
distances = sqrt(sum((nodes - target).^2, 2));

% Find the index of the closest node
[~, closestNodeIndex] = min(distances);

input_log_node = closestNodeIndex;

%% Plot dimensions
sim_params.plot_x = [-0.05,0.15];
sim_params.plot_y = [-0.05,0.15];
sim_params.plot_z = [-0.05,0.05];
