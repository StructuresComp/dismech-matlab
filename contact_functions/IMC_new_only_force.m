function [Fc, Ffr] = ...
    IMC_new_only_force(imc, q, q0, dt)
k_c = imc.k_c;
C = imc.C;
delta = imc.delta;
scale = imc.scale;
velTol = imc.velTol;
contact_len = imc.contact_len;
mu_k = imc.mu_k;
friction_present = imc.compute_friction;

n_dof = size(q,1);

if(~isempty(C))
    [colliding_edge_combos, ~,~] = detectCollisions(q, C, delta, contact_len, scale);

    [Fc, ~, Ffr, ~] = computeIMCContactAndFriction(q, q0, colliding_edge_combos, delta, contact_len, scale, k_c, mu_k, dt, velTol, n_dof, false, friction_present);

else 
    Fc = zeros(n_dof, 1);
    Ffr = zeros(n_dof, 1);
end
end
