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
% 
%     % Compute time parallel reference frame
%     [a1_iter, a2_iter] = computeTimeParallel(MultiRod, a1, q0, q);
% 
%     % Compute reference twist
%     tangent = computeTangent(MultiRod, q);
%     % if(~isempty(bend_twist_springs))
%         refTwist_iter = computeRefTwist_bend_twist_spring(bend_twist_springs, a1_iter, tangent, refTwist);
%     % end
%     % Compute material frame
%     theta = q(3*n_nodes + 1 : 3*n_nodes + n_edges_dof);
%     [m1_axis, m2_axis] = computeMaterialDirectors(a1_iter,a2_iter,theta);


    %% compute forces
    % Force calculation
    Forces = zeros(n_DOF,1);
    
    if(~isempty(bend_twist_springs))
    Fs = getFs(MultiRod, stretch_springs, q);
    Forces = Forces + Fs;
    end

    if(~isempty(bend_twist_springs))
    if(sim_params.TwoDsim)
        Fb = getFb(MultiRod, bend_twist_springs, q, m1, m2); % bending (rod)
        Ft = zeros(n_DOF,1);
    else
        Fb = getFb(MultiRod, bend_twist_springs, q, m1_axis, m2_axis); % bending (rod)
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

    % Add all elastic forces and jacobian
%     Forces = Fs + Fb + Ft + Fb_shell;


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
        [Fv,~] = getViscousForce_correctedJacobian(q,q0,sim_params.dt,env.eta,MultiRod);
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

    if(sim_params.static_sim) 
        f = - Forces; % Equations of motion
    else     
        f = MultiRod.MassMat / sim_params.dt * ( (q-q0)/ sim_params.dt - u) - Forces; % Inertial forces added
    end

    if ismember("selfContact", env.ext_force_list) % IMC
        [Fc, Ffr] = ...
            IMC_new_only_force(imc, q, q0, sim_params.dt);        
        f = f - Fc - Ffr;
    end

    if ismember("floorContact", env.ext_force_list) % floor contact
        [Fc_floor, Ffr_floor] = computeFloorContactAndFriction_only_force(imc, sim_params.dt, q, q0, n_nodes, n_DOF);
        f = f - Fc_floor - Ffr_floor;
    end
%%
    x = 0.5 * norm(f(MultiRod.freeDOF))^2;
    error = sum(abs(f(MultiRod.freeDOF)) ); % just to check
    slope = (x - x0) / a;

%     %         if(slope<0)
%     %             success = true;
%     %             continue;
%     %         end

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

