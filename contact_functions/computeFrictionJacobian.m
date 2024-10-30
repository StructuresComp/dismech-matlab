function Jfr = computeFrictionJacobian(data, contact_forces, contact_jacobian, mu, dt, vel_tol, friction_type, constraint_type)
    % Extract position data
    x1s = data(1:3);
    x1e = data(4:6);
    x2s = data(7:9);
    x2e = data(10:12);
    x1s0 = data(13:15);
    x1e0 = data(16:18);
    x2s0 = data(19:21);
    x2e0 = data(22:24);

    % Extract force data
    f1s = contact_forces(1:3);
    f1e = contact_forces(4:6);
    f2s = contact_forces(7:9);
    f2e = contact_forces(10:12);

    inputs = [x1s, x1e, x2s, x2e, x1s0, x1e0, x2s0, x2e0, f1s, f1e, f2s, f2e, mu, dt, vel_tol];
    
    if(friction_type == "Sticking")
        friction_partial_dfr_dx = friction_partial_dfr_dx1_func(inputs);
        friction_partial_dfr_dfc = friction_partial_dfr_dfc1_func(inputs);
    elseif(friction_type == "Sliding")
        friction_partial_dfr_dx = friction_partial_dfr_dx2_func(inputs);
        friction_partial_dfr_dfc = friction_partial_dfr_dfc2_func(inputs);
    else
        error("friction_type for Jacobian computation should be either Sticking or Sliding");
    end

    if constraint_type == "PointToPoint"
        friction_partial_dfr_dfc(:, 4:6) = zeros(12,3);  % remove the entries corresponding to x1e
        friction_partial_dfr_dfc(:, 10:12) = zeros(12,3); % remove the entries corresponding to x2e
    elseif constraint_type == "PointToEdge"
        friction_partial_dfr_dfc(:,4:6) = zeros(12,3); % remove the entries corresponding to x1e
    end

    
    Jfr = friction_partial_dfr_dfc*contact_jacobian + friction_partial_dfr_dx;
    assert(~anynan(Jfr),'IMC friction jacobian is not real (NaN).');


end