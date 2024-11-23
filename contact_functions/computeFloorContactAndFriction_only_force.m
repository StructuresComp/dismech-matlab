function [F_floorContact, F_floorFric] = ...
    computeFloorContactAndFriction_only_force(imc, dt, q, q0, n_nodes, n_dof)
delta = imc.delta_floor;
h = imc.h;
mu = imc.mu_floor;
velTol = imc.velTol;
floor_has_friction = imc.floor_has_friction;
floor_z = imc.floor_z;
contact_stiffness = imc.k_c_floor;

K1 = 15/delta;
F_floorContact = zeros(n_dof,1);
F_floorFric = zeros(n_dof,1);
for i = 1:n_nodes
    node_ind = mapNodetoDOF(i);
    ind = node_ind(3);
    dist = q(ind) - h - floor_z;
    if dist > delta
        continue;
    end
    
    v = exp(-K1 * dist);
    
    f = (-2 * v * log(v + 1)) / (K1 * (v + 1));
    if(isnan(f))
    assert(~isnan(f),'floor contact force is not real (NaN).');
    end

    f = f * contact_stiffness;

    F_floorContact(ind) = F_floorContact(ind) - f; 

    if(floor_has_friction)
        curr_node = q(node_ind(1:2));
        pre_node = q0(node_ind(1:2));
        [ffr, friction_type] = computeFloorFriction(curr_node, pre_node, f, mu, dt, velTol);

        if(friction_type=="ZeroVel") 
            continue;
        end
        idx = [node_ind(1), node_ind(2)];
        F_floorFric (idx) = F_floorFric (idx) - ffr;
    end

end
end
