clc
clear all
% add to path 
addpath util_functions/
addpath contact_functions/
addpath rod_dynamics/
addpath shell_dynamics/
addpath external_forces/
addpath logging/

%% parallel running of multiple simulations
writeOutput = true;
n_mesh_dense = 5;
n_mesh_type = 4;
mesh_dense_nos = [20,25,30,35,40,45,50,55,60,65];
mesh_types = ["equilateral" , "random" , "right" , "eq_algn"]; % type of mesh

results_all = zeros(n_mesh_dense,n_mesh_type); % to store the end_node final deflections for all sims

% make parfor loop for running multiple sims parallely
% mesh_density: [20,25,30,35,40,45,50,55,60,65]
% mesh_type:
% equilateral = 1
% random = 2
% right isoceles = 3
% equilateral aligned = 4

for mesh_dense = 2
    results = zeros(n_mesh_type,1);
    for mesh_type = 2

        %% input

%%
% input: robotDescription.m
sim_params = struct();
imc = struct();
geom = struct();
material = struct();
environment = struct();
        robotDescriptionShellCantilever
        
%         sim_params.static_sim = true;
%         sim_params.TwoDsim = true;
%         sim_params.use_midedge = true; % boolean var to decide on using midedge normal or
%         % hinge model for shell bending
%         sim_params.use_lineSearch = false;
%         sim_params.floor_present = false;
%         sim_params.showFloor = false;
%         sim_params.logStep = 1;
%         sim_params.log_data = true;
%         
%         % Time step
%         sim_params.dt = 1e-2;
%         
%         % gravity
%         gravity = 1; % or 0 for off
%         
%         if (gravity==1)
%             g = [0, 0, -9.81]';
%         else
%             g = [0, 0, 0]';
%         end
%         
%         sim_params.g = g;
%         
%         % Maximum number of iterations in Newton Solver
%         sim_params.maximum_iter = 25;
%         
%         % Total simulation time
%         if(sim_params.TwoDsim)
%             sim_params.totalTime = sim_params.dt;
%         else
%             sim_params.totalTime = 1; % sec
%         end
%         
%         % How often the plot should be saved? (Set plotStep to 1 to show each plot)
%         sim_params.plotStep = 1;
%         
%         %% Input parameters
%         % geometry parameters
%         geom.rod_r0 = 1e-3;
%         geom.shell_h = 1e-3;
%         
%         % material parameters
%         material.density = 1200;
%         material.youngs_rod = 0; % not used
%         material.youngs_shell = 2e8;
%         material.poisson_rod = 0;
%         material.poisson_shell = 0.5;
%         
%         % environment parameters
%         environment.mu = 0.25;
%         environment.eta = 0;
%         environment.Cd = 0;
%         environment.rho = 1;
%         
%         % point force
%         environment.ptForce = [0, 0, 0];
%         environment.ptForce_node = 1;
%         
%         % imc
%         imc.compute_friction = false;
%         imc.k_c = 100;
%         imc.k_c_floor = 100;
%         imc.contact_len = 2*geom.rod_r0;
%         imc.delta = 0.01*imc.contact_len;
%         imc.delta_floor = 0.05;
%         imc.omega = 20;
%         imc.h = geom.rod_r0;
%         imc.scale = 1/imc.h;
%         imc.C = [];
%         imc.mu_k = environment.mu;
%         imc.velTol = 1e-2;
%         imc.floor_has_friction = false;
%         imc.floor_z = -0.5;
%         
%         
%         %% Input text file
%         % inputFileName = 'experiments/shellCantilever/input_shell_cantilever_less_dense.txt';
%         % inputFileName = 'experiments/shellCantilever/equilateral_mesh_60.txt';
%         % inputFileName = 'experiments/shellCantilever/random_mesh_20.txt';
%         % inputFileName = 'experiments/shellCantilever/random_mesh_40.txt';
%         
%         
%         FileName = strcat(mesh_types(mesh_type), '_mesh_', num2str(mesh_dense_nos(mesh_dense)), '.txt');
%         inputFileName = strcat('experiments/shellCantilever/', FileName)
%         
%         % reading the input text file
%         [rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes] = inputProcessorNew(inputFileName);
%         
%         [nodes, edges, rod_nodes, shell_nodes, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, ...
%             elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
%             = createGeometry(rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, face_nodes);
%         
%         %% Tolerance on force function.
%         
%         sim_params.tol = 1e-4;
%         sim_params.ftol = 1e-4;
%         sim_params.dtol = 1e-2;
%         
%         %% Boundary conditions
%         fixed_node_indices = find(shell_nodes(:,1)<=0.01)';
%         fixed_edge_indices = [];
%         
%         for i=1:size(edges,1)
%             if ( ismember(edges(i,1),fixed_node_indices) && ismember(edges(i,2),fixed_node_indices) )
%                 fixed_edge_indices = [fixed_edge_indices, i];
%             end
%         end
%         
%         %% logging
%         % p = find(shell_nodes(:,1)==0.1)';
%         % input_log_node = p(1);
%         
%         p = find(shell_nodes(:,1)==0.1)';
%         if (isempty(p))
%             p = find(shell_nodes(1,:)>0.1);
%         end
%         Nodes_p = [shell_nodes(p,:)';p];
%         input_log_node = Nodes_p(4,find(Nodes_p(2,:) == 0));
%         
%         if (isempty(input_log_node))
%             input_log_node = Nodes_p(4,1);
%         end
%         
%         %% Plot dimensions
%         sim_params.plot_x = [0,0.1];
%         sim_params.plot_y = [-0.05,0.05];
%         sim_params.plot_z = [-0.05,0.05];

%%

        n_nodes = size(nodes,1);
        n_edges = size(edges,1);
        n_edges_rod_only = size(rod_edges,1);
        n_edges_shell_only = size(shell_edges,1);
        n_hinge = size(elBendShell,1);
        n_edges_rod_shell = size(rod_shell_joint_edges,1);
        n_edges_shell = size(shell_edges,1);
        twist_angles=zeros(n_edges_rod_only+n_edges_rod_shell,1);

        %% Create the soft robot structure
        softRobot = MultiRod(geom, material, twist_angles,...
            nodes, edges, rod_nodes, shell_nodes, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, ...
            face_nodes, sign_faces, face_edges, sim_params);

        %% Creating stretching, bending, twisting springs

        n_stretch = size(elStretchRod,1) + size(elStretchShell,1);
        n_bend_twist = size(elBendRod,1);

        % stretching spring
        if(n_stretch==0)
            stretch_springs = [];
        else
            for s=1:n_stretch
                if (s <= size(elStretchRod,1)) % rod
                    stretch_springs (s) = stretchSpring (...
                        softRobot.refLen(s), elStretchRod(s,:),softRobot);

                else % shell
                    stretch_springs (s) = stretchSpring (...
                        softRobot.refLen(s), ...
                        elStretchShell(s-(size(elStretchRod,1)),:), ...
                        softRobot, softRobot.ks(s));
                end
            end
        end

        % bending and twisting spring
        if(n_bend_twist==0)
            bend_twist_springs = [];
        else
            for b=1:n_bend_twist
                bend_twist_springs(b) = bendTwistSpring ( ...
                    elBendRod(b,:), elBendSign(b,:), [0 0], 0, softRobot);
            end
        end

        % hinge bending spring
        if(n_hinge==0)
            hinge_springs = [];
        else
            if(~sim_params.use_midedge)
                for h=1:n_hinge
                    hinge_springs(h) = hingeSpring (...
                        0, elBendShell(h,:), softRobot, softRobot.kb);
                end
                hinge_springs = setThetaBar(hinge_springs, softRobot);
            else
                hinge_springs = [];
            end
        end

        %% Prepare system
        % Reference frame (Space parallel transport at t=0)
        softRobot = computeSpaceParallel(softRobot);

        % Material frame from reference frame and twist angle
        theta = softRobot.q0(3*softRobot.n_nodes+1:3*softRobot.n_nodes+softRobot.n_edges_dof); % twist angle
        [softRobot.m1, softRobot.m2] = computeMaterialDirectors(softRobot.a1,softRobot.a2,theta);

        % Set rod natural curvature
        bend_twist_springs = setkappa(softRobot, bend_twist_springs);

        % Reference twist
        undef_refTwist = computeRefTwist_bend_twist_spring ...
            (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
            zeros(n_bend_twist,1));
        refTwist = computeRefTwist_bend_twist_spring ...
            (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
            zeros(n_bend_twist,1));
        softRobot.refTwist = refTwist;
        softRobot.undef_refTwist = undef_refTwist;

        %% Boundary Conditions

        softRobot.fixed_nodes = fixed_node_indices;
        softRobot.fixed_edges = fixed_edge_indices;
        [softRobot.fixedDOF, softRobot.freeDOF] = FindFixedFreeDOF(softRobot.fixed_nodes, softRobot.fixed_edges, softRobot.n_DOF, softRobot.n_nodes);

        % Visualize initial configuration and the fixed and free nodes: free nodes - blue, fixed - red
        plot_MultiRod(softRobot, 0.0, sim_params);

        %% initial conditions on velocity / angular velocity (if any)


        %% Time stepping scheme

        Nsteps = round(sim_params.totalTime/sim_params.dt);
        ctime = 0; % current time
        current_pos = zeros(Nsteps,1);
        time_arr = linspace(0,sim_params.totalTime,Nsteps);

        log_node = input_log_node;
        current_pos(1) = softRobot.q0(3*log_node);

        dof_with_time = zeros(softRobot.n_DOF+1,Nsteps);
        dof_with_time(1,:) = time_arr;

        for timeStep = 1:Nsteps
            %% Precomputation at each timeStep: midedge normal shell bending
            if(sim_params.use_midedge)
                tau_0 = updatePreComp_without_sign(softRobot.q, softRobot);
            else
                tau_0 = [];
            end

            %%  DER error iteration
            [softRobot, stretch_springs, bend_twist_springs, hinge_springs] = ...
                DERfun(softRobot, stretch_springs, bend_twist_springs, hinge_springs, tau_0,environment,imc, sim_params);

            ctime = ctime + sim_params.dt

            % Update q
            softRobot.q0 = softRobot.q;

            % log values
            current_pos(timeStep) = softRobot.q0(3*log_node);

            if(sim_params.log_data)
                if mod(timeStep, sim_params.logStep) == 0
                    dof_with_time(2:end,timeStep) =  softRobot.q;
                end
            end

            %% Saving the coordinates and Plotting

            if mod(timeStep, sim_params.plotStep) == 0
                % visualisation
                plot_MultiRod(softRobot, ctime, sim_params);
            end
        end
        % save the current position (end-1) to results
        results(mesh_type) = current_pos;

        % [rod_data,shell_data] = logDataForRendering(dof_with_time, MultiRod, Nsteps);
        clearvars -except results mesh_type mesh_dense mesh_dense_nos mesh_types n_mesh_type n_mesh_dense results_all writeOutput

    end % multiple test simulations for loop end

    % store results in results_all array which is accessible outside of the
    % parfor loop
    results_all (mesh_dense,:) = results;
end % multiple test simulations parfor loop end

if(writeOutput)
    filename = 'hinge_cantilever_next.xlsx';
    writematrix(results_all, filename, Sheet=1);
end