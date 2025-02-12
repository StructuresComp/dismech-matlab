function [Fc, Jc, Ffr, Jfr, imc] = ...
    IMC_new(imc, q, q0, edge_combos, iter, dt, fvec_exceptIMC, fixedDOF)

k_c = imc.k_c;
C = imc.C;
delta = imc.delta;
omega = imc.omega;
scale = imc.scale;
velTol = imc.velTol;
contact_len = imc.contact_len;
mu_k = imc.mu_k;
friction_present = imc.compute_friction;


col_lim = 100*delta;
candidate_lim = scale*(contact_len + col_lim);
n_dof = size(q,1);

if(iter==1) % run only on first iter
    [C, ~] = constructCandidateSet(q, edge_combos, candidate_lim, scale);
    if(~isempty(C)) % if collision is detected, update contact stiffness if necessary
        k_c = updateContactStiffnessNew(fvec_exceptIMC, C, fixedDOF);
    end
end
if(~isempty(C))
    [colliding_edge_combos, ~,~] = detectCollisions(q, C, delta, contact_len, scale);

    use_hess = false; % if iter<omega compute only forces
    if (iter>omega) 
        use_hess = true; % compute Jacobian for convergence
    end
    [Fc, Jc, Ffr, Jfr] = computeIMCContactAndFriction(q, q0, colliding_edge_combos, delta, contact_len, scale, k_c, mu_k, dt, velTol, n_dof, use_hess, friction_present);
else 
    Fc = zeros(n_dof, 1);
    Jc = zeros(n_dof, n_dof);
    Ffr = zeros(n_dof, 1);
    Jfr = zeros(n_dof, n_dof);
end
imc.C= C;
imc.k_c = k_c;

end
