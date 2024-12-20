function Jfr = computeFloorFrictionJacobian_custom_gd(curr_node, pre_node, fn, contact_jacobian, mu, dt, velTol, friction_type, n_gd)
    % Extract position data
    x1s_x = curr_node(1);
    x1s_y = curr_node(2);
    x1s_z = curr_node(3);
    x1s_x0 = pre_node(1);
    x1s_y0 = pre_node(2);
    x1s_z0 = pre_node(3);

    K2 = 15/velTol;
    inputs = [x1s_x, x1s_y, x1s_z, x1s_x0, x1s_y0, x1s_z0, fn', mu, dt, K2, n_gd'];

    if(friction_type == "Sticking")
        floor_friction_partial_dfr_dx = floor_friction_partial_dfr_dx_func_custom_gd(inputs);
        floor_friction_partial_dfr_dfc = floor_friction_partial_dfr_dfn_func_custom_gd(inputs);
    elseif(friction_type == "Sliding")
        floor_friction_partial_dfr_dx = floor_friction_g1_partial_dfr_dx_func_custom_gd(inputs);
        floor_friction_partial_dfr_dfc = floor_friction_g1_partial_dfr_dfn_func_custom_gd(inputs);
    else
        error("friction_type for Jacobian computation should be either Sticking or Sliding");
    end
    
    Jfr = floor_friction_partial_dfr_dfc*contact_jacobian + floor_friction_partial_dfr_dx;
    assert(~anynan(Jfr),'IMC friction jacobian is not real (NaN).');


end