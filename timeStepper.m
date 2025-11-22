function [MultiRod, stretch_springs, bend_twist_springs, hinge_springs] = ...
    timeStepper (MultiRod, stretch_springs, bend_twist_springs, hinge_springs, triangle_springs, tau_0, env, imc, sim_params)

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

alpha = 1;

% Guess: new DOF is same as old DOF vector
q = q0;
iter = 1; % Number of iterations
err = 10 * sim_params.tol;
error0 = err;
solved = false;
while ~solved % % error > sim_params.tol

    % intialize force and Jacobian
    Forces = zeros(n_DOF,1);
    JForces = zeros(n_DOF,n_DOF);

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
    [Fs, Js, stretch_springs] = getFsJs(MultiRod, stretch_springs, q); % stretching

    Forces = Forces + Fs;
    JForces = JForces + Js;
    end

    if(~isempty(bend_twist_springs))
        if(sim_params.TwoDsim)
            if(isfield(sim_params, 'FDM'))
                if(sim_params.FDM)
                    [Fb, Jb, ~,~, ~,~, bend_twist_springs] = getFbJb_FtJt_and_FDM(MultiRod, bend_twist_springs, q, m1, m2, refTwist_iter, sim_params); % bending (rod)          
                else
                    [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2, sim_params); % bending (rod)
                end
            else

                [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2, sim_params); % bending (rod)
            end

            Forces = Forces + Fb;
            JForces = JForces + Jb;
        else

            if(isfield(sim_params, 'FDM'))
                if(sim_params.FDM)
                    [Fb, Jb, Ft, Jt, Jb_FDM, Jt_FDM, bend_twist_springs] = getFbJb_FtJt_and_FDM(MultiRod, bend_twist_springs, q, m1, m2, refTwist_iter, sim_params); % bending (rod)          
                else
                    [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2, sim_params); % bending (rod)
                    [Ft, Jt, bend_twist_springs] = getFtJt(MultiRod, bend_twist_springs, q, refTwist_iter, sim_params); % twisting
                end
            else
                [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2, sim_params); % bending (rod)
                [Ft, Jt, bend_twist_springs] = getFtJt(MultiRod, bend_twist_springs, q, refTwist_iter, sim_params); % twisting

            end
            Forces = Forces + Fb + Ft;
            JForces = JForces + Jb + Jt;
        end
    end

    if(~isempty(MultiRod.face_nodes_shell))
        if (sim_params.use_midedge)
            [Fb_shell, Jb_shell] = getFbJb_shell_midedge(MultiRod, triangle_springs, q, tau_0); % midedge-bending (shell)
        else
            [Fb_shell, Jb_shell, hinge_springs] = getFbJb_shell(MultiRod, hinge_springs, q); % hinge-bending (shell)
        end
        Forces = Forces + Fb_shell;
        JForces = JForces + Jb_shell;
        
    end

    %% External force and Jacobian calculation
    if ismember("gravity",env.ext_force_list) % Gravity 
        if(sim_params.static_sim)
            Fg = getGravityForce(MultiRod, env);
        else
            Fg = MultiRod.Fg;
        end
        Forces = Forces + Fg;
    end

    if ismember("viscous", env.ext_force_list) % Viscous forces
        % [Fv,Jv] = getViscousForce(q,q0,sim_params.dt,env.eta,MultiRod);
        [Fv,Jv] = getViscousForce_temp(q,q0,sim_params.dt,env.eta,MultiRod);

        Forces = Forces + Fv;
        JForces = JForces + Jv;
    end

    if ismember("aerodynamic", env.ext_force_list) % Aerodynamic drag
        [Fd, Jd] = getAerodynamicDrag(q,q0,sim_params.dt,env,MultiRod);
        Forces = Forces + Fd;
        JForces = JForces + Jd;
    end

    if ismember("pointForce", env.ext_force_list) % Point force
        Fpt = addPointForce(env.ptForce, env.ptForce_node, MultiRod);
        Forces = Forces + Fpt;
    end

    if ismember("rft", env.ext_force_list)
        [Frft,Jrft] = get_rft_force(MultiRod, q, u, env, sim_params);
        Forces = Forces + Frft;
        JForces = JForces + Jrft;
    end

    if(sim_params.static_sim) 
        f = - Forces; % Equations of motion
        J = - JForces; % Jacobian
    else     
        f = MultiRod.MassMat / sim_params.dt * ( (q-q0)/ sim_params.dt - u) - Forces; % Inertial forces added
        J = MultiRod.MassMat / sim_params.dt^2 - JForces; % Inertial Jacobian added
    end

    if ismember("selfContact", env.ext_force_list) % IMC
        [Fc, Jc, Ffr, Jfr, imc] = ...
            IMC_new(imc, q, q0, MultiRod.edge_combos, iter, sim_params.dt, f, MultiRod.fixedDOF);
        f = f - Fc - Ffr;
        J = J - Jc - Jfr;
    end

    if ismember("floorContact", env.ext_force_list) % floor contact
        [Fc_floor,Jc_floor, Ffr_floor, Jfr_floor] = computeFloorContactAndFriction_custom_ground(imc, sim_params.dt, q, q0, n_nodes, n_DOF);
        % [Fc_floor,Jc_floor, Ffr_floor, Jfr_floor] = computeFloorContactAndFriction_sphere_gd(imc, sim_params.dt, q, q0, n_nodes, n_DOF);

        f = f - Fc_floor - Ffr_floor;
        J = J - Jc_floor - Jfr_floor;
    end

    f_free = f(freeIndex);
    J_free = J(freeIndex, freeIndex);
    Det_J = det(J_free); % to debug

    % J\f solving
    if(Det_J==0 && sum(abs(f_free)) <= 1e-8) % if the force is zero and Jacobian is singular
        dq_free = zeros(numel(freeIndex),1);
    else
        dq_free = J_free \ f_free;
    end

    dq = zeros(n_DOF,1);
    dq(freeIndex) = dq_free;

    % lineSearch for optimal alpha
    if(sim_params.use_lineSearch && iter>10)
        alpha = lineSearch(q,q0,dq,u,f,J, stretch_springs, bend_twist_springs, hinge_springs, MultiRod, tau_0, imc, env, sim_params);
    else
        alpha = newtonDamper(alpha,iter);
    end

    % Newton's update
    q(freeIndex) = q(freeIndex) - alpha.*dq_free;

    % Error
    err = norm(f_free) ;
    fprintf('Iter=%d, error=%f\n', iter, err);

    if(iter==1)
        error0=err;
    end

    if(err<=sim_params.tol)
        solved = true;
        continue;
    end

    if(err<error0*sim_params.ftol)
        solved = true;
        continue;
    end

    if(max(abs(dq(1:3*n_nodes)))/sim_params.dt < sim_params.dtol)
        solved = true;
        continue;
    end

    if(iter>sim_params.maximum_iter)
        error("Could not converge in iters; exiting simulation");
    end

    iter = iter + 1;

end
a1 = a1_iter;
a2 = a2_iter;
u = (q - q0) / sim_params.dt;

%% update
MultiRod.q=q;
MultiRod.u=u;
MultiRod.a1=a1;
MultiRod.a2=a2;
MultiRod.m1 = m1;
MultiRod.m2 = m2;
MultiRod.refTwist = refTwist_iter;

end
