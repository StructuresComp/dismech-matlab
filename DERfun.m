function [MultiRod, stretch_springs, bend_twist_springs, hinge_springs] = ...
    DERfun (MultiRod, stretch_springs, bend_twist_springs, hinge_springs, tau_0, env, imc, sim_params)

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
error = 10 * sim_params.tol;
error0 = error;
solved = false;
while ~solved % error > sim_params.tol

    % Compute time parallel reference frame
    [a1_iter, a2_iter] = computeTimeParallel(MultiRod, a1, q0, q);

    % Compute reference twist
    tangent = computeTangent(MultiRod, q);
    refTwist_iter = computeRefTwist_bend_twist_spring(bend_twist_springs, a1_iter, tangent, refTwist);

    % Compute material frame
    theta = q(3*n_nodes + 1 : 3*n_nodes + n_edges_dof);
    [m1, m2] = computeMaterialDirectors(a1_iter,a2_iter,theta);

    %% Elastic force and jacobian calculation
    [Fs, Js, stretch_springs] = getFsJs(MultiRod, stretch_springs, q);
    if(sim_params.TwoDsim)
        [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2);
        Ft = zeros(n_DOF,1);
        Jt= zeros(n_DOF,n_DOF);

    else
        [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2);
        [Ft, Jt, bend_twist_springs] = getFtJt(MultiRod, bend_twist_springs, q, refTwist_iter);
    end

    if (sim_params.use_midedge)
        [Fb_shell, Jb_shell] = getFbJb_shell_midedge(MultiRod, q, tau_0);
    else
        [Fb_shell, Jb_shell, hinge_springs] = getFbJb_shell(MultiRod, hinge_springs, q);
    end

    %% External force and Jacobian calculation
    % Gravity 
    if(sim_params.static_sim)
        Fg = getGravityForce(MultiRod, env);
    else
        Fg = MultiRod.Fg;
    end

    % Viscous forces
    [Fv,Jv] = getViscousForce_correctedJacobian(q,q0,sim_params.dt,env.eta,MultiRod);

    % Aerodynamic drag
    [Fd, Jd] = getAerodynamicDrag(q,q0,sim_params.dt,env,MultiRod);

    Forces = Fs + Fb + Ft + Fb_shell + Fv + Fd + Fg;
    f = MultiRod.MassMat / sim_params.dt * ( (q-q0)/ sim_params.dt - u) - Forces;
    
    % IMC
    [Fc, Jc, Ffr, Jfr, imc] = ...
        IMC_new(imc, q, q0, MultiRod.edge_combos, iter, sim_params.dt, f, MultiRod.fixedDOF);

    % floor contact
    if(sim_params.floor_present)
        [Fc_floor,Jc_floor, Ffr_floor, Jfr_floor] = computeFloorContactAndFriction(imc, sim_params.dt, q, q0, n_nodes, n_DOF);
    else
        Fc_floor = zeros(n_DOF,1);
        Jc_floor = zeros(n_DOF,n_DOF);
        Ffr_floor = zeros(n_DOF,1);
        Jfr_floor = zeros(n_DOF,n_DOF);
    end
    % Add external point force
    Fpt = addPointForce(env.ptForce, env.ptForce_node, MultiRod);

%%
    Forces = Forces + Fc + Ffr + Fc_floor + Ffr_floor + Fpt;
    JForces = Js + Jb + Jt + Jb_shell + Jv + Jd + Jc + Jfr + Jc_floor + Jfr_floor;

    if(sim_params.static_sim)
        % Equations of motion
        f = - Forces;

        % Jacobian
        J = - JForces;

    else
        % Equations of motion
        f = MultiRod.MassMat / sim_params.dt * ( (q-q0)/ sim_params.dt - u) - Forces;

        % Jacobian
        J = MultiRod.MassMat / sim_params.dt^2 - JForces;
    end

    f_free = f(freeIndex);
    J_free = J(freeIndex, freeIndex);
    Det_J = det(J_free); % to debug (takes very high values when the simulation starts to crash)

    % Newton's update
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
         error = sum(abs(f_free) );
%     error = norm(f_free) ;
    fprintf('Iter=%d, error=%f\n', iter, error);

    if(iter==1)
        error0=error;
    end

    if(error<=sim_params.tol)
        solved = true;
        continue;
    end

    if(error<error0*sim_params.ftol)
        solved = true;
        continue;
    end

    if(max(abs(dq(1:3*n_nodes)))/sim_params.dt < sim_params.dtol)
        solved = true;
        continue;
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
