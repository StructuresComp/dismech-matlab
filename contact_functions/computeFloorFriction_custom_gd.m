function [ffr, friction_type] = computeFloorFriction_custom_gd(curr_node, pre_node, fn, mu, dt, velTol, n_gd)
    K2 = 15/velTol;

    % Calculate velocity vector
    v = (curr_node - pre_node) / dt;

    % Calculate tangential velocity
    v = v - dot(v,n_gd)*n_gd;

    v_n = norm(v);
    if(v_n==0) % avoid /0 if there is no relative tangential velocity upon contact
        v_hat = [0,0,0];
    else
        v_hat = v / v_n;
    end

    % Initialize friction force
    ffr = zeros(3,1);

    % Determine friction type and gamma based on velocity magnitude
    if v_n == 0
        friction_type = "ZeroVel";
        return;
    elseif v_n > velTol % if sliding
        friction_type = "Sliding";
        gamma = 1;
    else % if sticking
        friction_type = "Sticking";
        gamma = 2 / (1 + exp(-K2 * v_n)) - 1;
    end

    % Calculate friction force
    ffr = -(gamma * mu * fn) .* v_hat;
end