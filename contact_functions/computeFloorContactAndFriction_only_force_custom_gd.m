function [F_floorContact, F_floorFric] = ...
    computeFloorContactAndFriction_only_force_custom_gd(imc, dt, q, q0, n_nodes, n_dof)
delta = imc.delta_floor;
h = imc.h;
floor_has_friction = imc.floor_has_friction;
floor_z = imc.floor_z;
contact_stiffness = imc.k_c_floor;

K1 = 15/delta;
F_floorContact = zeros(n_dof,1);
F_floorFric = zeros(n_dof,1);
for i = 1:n_nodes
    ind = mapNodetoDOF(i);
    dist = q(ind(3)) - h - floor_z;
    n_gd = [0;0;1];

    % dist = min_dist_pt_gd(); % replace with custom functions
    % n_gd = normal_at_contact(); % replace with custom functions

    if dist > delta
        continue;
    end
    
    v = exp(-K1 * dist);
    
    f = (-2 * v * log(v + 1)) / (K1 * (v + 1)) * n_gd;
    if(isnan(f))
    assert(~isnan(f),'floor contact force is not real (NaN).');
    end

    f = f * contact_stiffness;

    F_floorContact(ind) = F_floorContact(ind) - f; 

    if(floor_has_friction)
        curr_node = q(ind);
        pre_node = q0(ind);
        f_con = norm(f);
        [ffr, friction_type] = computeFloorFriction_custom_gd(curr_node, pre_node, f_con, imc.mu_floor, dt, imc.velTol, n_gd);

        if(friction_type=="ZeroVel") 
            continue;
        end
        F_floorFric (ind) = F_floorFric (ind) + ffr;
    end

end
end
