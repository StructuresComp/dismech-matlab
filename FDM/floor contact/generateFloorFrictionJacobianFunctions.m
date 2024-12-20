function [floor_friction_partial_dfr_dx_custom_gd_func, floor_friction_partial_dfr_dfn_custom_gd_func, ...
    floor_friction_g1_partial_dfr_dx_custom_gd_func, floor_friction_g1_partial_dfr_dfn_custom_gd_func] = ...
    generateFloorFrictionJacobianFunctions()
    
    % Define symbolic variables
    syms x1s_x x1s_y x1s_z x1s_x0 x1s_y0 x1s_z0 mu dt K2 real
    syms n_gd [3 1] real
    syms fn [3 1] real

    % Symbolic node and node_0
    node = [x1s_x; x1s_y; x1s_z];
    node_0 = [x1s_x0; x1s_y0; x1s_z0];

    % Compute velocity v = (node - node_0) / dt
    v = (node - node_0) / dt;    
    v = v - dot(v,n_gd)*n_gd;
    v_n = norm (v);
    v_hat = v / v_n;

    % Calculate gamma and friction force scalar (sticking friction) 
    v_n_scaled = K2 * v_n;
    gamma = 2 / (1 + exp(-v_n_scaled)) - 1;
    fn_norm = norm(fn);
    ffr_scalar = gamma * mu * fn_norm;

    % Calculate friction force vector
    ffr = -ffr_scalar * v_hat;

    % Inputs for the jacobian calculation
    inputs = [x1s_x, x1s_y, x1s_z, x1s_x0, x1s_y0, x1s_z0, fn', mu, dt, K2, n_gd'];

    % Compute the Jacobian matrices
    floor_friction_partial_dfr_dx_custom_gd = jacobian(ffr, node);
    floor_friction_partial_dfr_dfn_custom_gd = jacobian(ffr, fn);

    % Now, assume gamma = 1 (sliding friction)
    ffr_scalar = mu * fn_norm;
    ffr = -ffr_scalar * v_hat;

    % Compute the Jacobians for the case where gamma = 1
    floor_friction_g1_partial_dfr_dx_custom_gd = jacobian(ffr, node);
    floor_friction_g1_partial_dfr_dfn_custom_gd = jacobian(ffr, fn);

    % Convert to different MATLAB functions
    floor_friction_partial_dfr_dx_custom_gd_func = matlabFunction(floor_friction_partial_dfr_dx_custom_gd, 'Vars', {inputs}, 'File', 'floor_friction_partial_dfr_dx_func_custom_gd.m');
    floor_friction_partial_dfr_dfn_custom_gd_func = matlabFunction(floor_friction_partial_dfr_dfn_custom_gd, 'Vars', {inputs}, 'File', 'floor_friction_partial_dfr_dfn_func_custom_gd.m');
    floor_friction_g1_partial_dfr_dx_custom_gd_func = matlabFunction(floor_friction_g1_partial_dfr_dx_custom_gd, 'Vars', {inputs}, 'File', 'floor_friction_g1_partial_dfr_dx_func_custom_gd.m');
    floor_friction_g1_partial_dfr_dfn_custom_gd_func = matlabFunction(floor_friction_g1_partial_dfr_dfn_custom_gd, 'Vars', {inputs}, 'File', 'floor_friction_g1_partial_dfr_dfn_func_custom_gd.m');
end
