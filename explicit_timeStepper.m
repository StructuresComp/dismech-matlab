function [MultiRod, stretch_springs, bend_twist_springs, hinge_springs] = ...
    explicit_timeStepper (MultiRod, stretch_springs, bend_twist_springs, hinge_springs, triangle_springs, tau_0, env, imc, sim_params)

% create local variables in function to store the struct values
n_nodes=MultiRod.n_nodes;
n_edges = MultiRod.n_edges;
n_DOF=MultiRod.n_DOF;
n_edges_dof = MultiRod.n_edges_dof;
q0=MultiRod.q0;
u=MultiRod.u;
a1=MultiRod.a1;
a2=MultiRod.a2;
freeIndex=MultiRod.freeDOF;
refTwist=MultiRod.refTwist;

% Guess: new DOF is same as old DOF vector
    q = q0;
    % intialize force and Jacobian
    Forces = zeros(n_DOF,1);

    %% prepare for iterations
    % Compute time parallel reference frame
    [a1_iter, a2_iter] = computeTimeParallel(MultiRod, a1, q0, q);

    % Compute reference twist
    tangent = computeTangent(MultiRod, q);
    refTwist_iter = computeRefTwist_bend_twist_spring(bend_twist_springs, a1_iter, tangent, refTwist);

    % Compute material frame
    theta = q(3*n_nodes + 1 : 3*n_nodes + n_edges_dof);
    [m1, m2] = computeMaterialDirectors(a1_iter,a2_iter,theta);

    %% Elastic force and jacobian calculation
    
    if(~isempty(stretch_springs))
    Fs = getFs(MultiRod, stretch_springs, q);
    Forces = Forces + Fs;
    end

    if(~isempty(bend_twist_springs))
    if(sim_params.TwoDsim)
        Fb = getFb(MultiRod, bend_twist_springs, q, m1, m2); % bending (rod)
        Ft = zeros(n_DOF,1);
    else
        Fb = getFb(MultiRod, bend_twist_springs, q, m1, m2); % bending (rod)
        Ft = getFt(MultiRod, bend_twist_springs, q, refTwist_iter); % twisting
    end
    Forces = Forces + Fb + Ft;
    end

    if(~isempty(MultiRod.face_nodes_shell))
        if (sim_params.use_midedge)
            Fb_shell = getFb_shell_midedge(MultiRod, q, tau_0); % hinge-bending (shell)
        else
            Fb_shell = getFb_shell(MultiRod, hinge_springs, q); % midedge-bending (shell)
        end
        Forces = Forces + Fb_shell;
    end

    %% External force calculation
    
    if ismember("gravity",env.ext_force_list) % Gravity 
        if(sim_params.static_sim)
            Fg = getGravityForce(MultiRod, env);
        else
            Fg = MultiRod.Fg;
        end
        Forces = Forces + Fg;
    end

    if ismember("viscous", env.ext_force_list) % Viscous forces
        Fv = getViscousForce_explicit(u,sim_params.dt,env.eta,MultiRod);
        Forces = Forces + Fv;
    end

    if ismember("aerodynamic", env.ext_force_list) % Aerodynamic drag
        [Fd, ~] = getAerodynamicDrag(q,q0,sim_params.dt,env,MultiRod);
        Forces = Forces + Fd;
    end

    if ismember("pointForce", env.ext_force_list) % Point force
        Fpt = addPointForce(env.ptForce, env.ptForce_node, MultiRod);
        Forces = Forces + Fpt;
    end

    if ismember("selfContact", env.ext_force_list) % IMC
        [Fc, Ffr] = ...
            IMC_new_only_force(imc, q, q0, sim_params.dt);        
        Forces = Forces + Fc + Ffr;
    end

    if ismember("floorContact", env.ext_force_list) % floor contact
        [Fc_floor, Ffr_floor] = computeFloorContactAndFriction_only_force_custom_gd(imc, sim_params.dt, q, q0, n_nodes, n_DOF);
        Forces = Forces + Fc_floor + Ffr_floor;
    end
    % NOTE: Did not add inertial forces, those are added in the explicit
    % step itself below
    accel = MultiRod.MassMat\Forces; % Forces./mass_vector
    % accel = Forces./MultiRod.massVec;
    q(freeIndex) = (sim_params.dt).*(accel(freeIndex).*sim_params.dt + u(freeIndex)) + q0(freeIndex);

%% update
MultiRod = update_system(MultiRod, bend_twist_springs, sim_params, a1, q0, q, refTwist);

end
