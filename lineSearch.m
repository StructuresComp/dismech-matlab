function [alpha] = lineSearch(q,q0,dq,u,f,J, stretch_springs, bend_twist_springs, hinge_springs, MultiRod, tau_0, imc, env, sim_params)
    % Store current q
    q_old = q;
    n_nodes=MultiRod.n_nodes;
    n_edges_dof = MultiRod.n_edges_dof;
    n_DOF=MultiRod.n_DOF;
    
    a1=MultiRod.a1;
    refTwist=MultiRod.refTwist;


    % Initialize an interval for optimal learning rate alpha
    amax = 2;
    amin = 1e-3;
    al = 0;
    au = 1;

    a = 1;

    % Compute the slope initially
    x0 = 0.5 * norm(f(MultiRod.freeDOF))^2; % Should decrease with the updated learning rate from lineSearch
    dx0 = -(transpose(f(MultiRod.freeDOF)) * J(MultiRod.freeDOF,MultiRod.freeDOF) * dq(MultiRod.freeDOF));

    success = false;
    m2 = 0.9;
    m1 = 0.1;
    iter_l = 1;

    while ~success
        %% NewtonUpdate
        q(MultiRod.freeDOF) = q_old(MultiRod.freeDOF)-a*dq(MultiRod.freeDOF);
        %% prepSystemForIteration

        % Compute time parallel reference frame
        [a1_iter, a2_iter] = computeTimeParallel(MultiRod, a1, q0, q);
    
        % Compute reference twist
        tangent = computeTangent(MultiRod, q);
        if(~isempty(bend_twist_springs))
            refTwist_iter = computeRefTwist_bend_twist_spring(bend_twist_springs, a1_iter, tangent, refTwist);
        end
        % Compute material frame
        theta = q(3*n_nodes + 1 : 3*n_nodes + n_edges_dof);
        [m1_axis, m2_axis] = computeMaterialDirectors(a1_iter,a2_iter,theta); 
    

        %% compute forces
        % Force calculation
        Fs = getFs(MultiRod, stretch_springs, q);
        if(~isempty(bend_twist_springs))
            Fb = getFb(MultiRod, bend_twist_springs, q, m1_axis, m2_axis);
            Ft = getFt(MultiRod, bend_twist_springs, q, refTwist_iter);
        else 
            Fb = zeros(n_DOF,1);
            Ft = zeros(n_DOF,1);
        end
        if (sim_params.use_midedge)
            Fb_shell = getFb_shell_midedge(MultiRod, q, tau_0);
        else
            Fb_shell = getFb_shell(MultiRod, hinge_springs, q);
        end
        % Viscous forces
        % [Fv,Jv] = getViscousForce(q,q0,sim_params.dt,env.eta,MultiRod);
        [Fv,~] = getViscousForce_newest(q,q0,sim_params.dt,env.eta,MultiRod);
        % Aerodynamic drag
        [Fd, ~] = getAerodynamicDrag_newest(q,q0,sim_params.dt,env,MultiRod);

        [Fc, Ffr] = ...
            IMC_new_only_force(imc, q, q0, sim_params.dt);
        % floor contact
        if(sim_params.floor_present)
              [Fc_floor, Ffr_floor] = computeFloorContactAndFriction_only_force(imc, sim_params.dt, q, q0, n_nodes, n_DOF);
        else
            Fc_floor = zeros(n_DOF,1);   
            Ffr_floor = zeros(n_DOF,1);
        end
        % Add external point force
        Fpt = addPointForce(env.ptForce, env.ptForce_node, MultiRod);
        
     %% Net forces
        Forces = Fs + Fb + Ft + Fb_shell + Fv + Fd+ Fc + Ffr + Fc_floor + Ffr_floor + MultiRod.W + Fpt;
        %  Forces = Fs + Fb + Ft + Fb_shell + Fc_floor + Ffr_floor + MultiRod.W;
        f = MultiRod.MassMat / sim_params.dt * ( (q-q0)/ sim_params.dt - u) - Forces;

        x = 0.5 * norm(f(MultiRod.freeDOF))^2;
        error = sum(abs(f(MultiRod.freeDOF)) ); % just to check
        slope = (x - x0) / a;

%         if(slope<0)
%             success = true;
%             continue;
%         end

        if isnan(x)
            error("x is NaN");
        end

        if slope >= m2 * dx0 && slope <= m1 * dx0
            success = true;
        else
            if slope < m2 * dx0
                al = a;
            else
                au = a;
            end

            if au < amax
                a = 0.5 * (al + au);
            else 
                a = 10 * a;
            end
        end

        if a > amax || a < amin || iter_l > 100
            break;
        end

        iter_l = iter_l + 1;
    end
    alpha = a;
end

