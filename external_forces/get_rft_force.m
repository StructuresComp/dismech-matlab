function [F_full, J_full] = get_rft_force(softRobot, q, u, env, sim_params)

    % Slice q and u for nodal DoFs
    q_n = q(1:softRobot.n_nodes*3);
    u_n = u(1:softRobot.n_nodes*3);

    % Compute N (number of nodes)
    N = numel(q_n) / 3;

    % Reshape into N x 3 matrices
    X = reshape(q_n, [3, N])';
    V = reshape(u_n, [3, N])';

    % Compute tangents
    T = zeros(N, 3);
    T(1, :)     = X(2, :) - X(1, :);
    T(N, :)     = X(N, :) - X(N-1, :);
    T(2:N-1, :) = X(3:N, :) - X(1:N-2, :);

    % Normalize tangents safely
    norms = sqrt(sum(T.^2, 2));
    norms(norms < 1e-12) = 1.0;
    T = T ./ norms;

    % Compute local resistance matrices
    ct = env.ct;
    cn = env.cn;
    dt = sim_params.dt;

    % Allocate M as 3x3 matrices for each node
    M = zeros(3, 3, N);
    for i = 1:N
        Ti = T(i, :)';
        M(:, :, i) = (ct - cn) * (Ti * Ti') + cn * eye(3);
    end

    % Compute forces
    F_nodes = zeros(N, 3);
    for i = 1:N
        F_nodes(i, :) = - M(:, :, i) * V(i, :)';
    end

    F_nodes = reshape(F_nodes.', [], 1); % column vector

    % Compute Jacobian blocks
    J_blocks = zeros(3, 3, N);
    for i = 1:N
        J_blocks(:, :, i) = - M(:, :, i) / dt;
    end

    % Assemble block diagonal Jacobian
    J_nodes = zeros(3*N, 3*N);
    for i = 1:N
        idx = (3*(i-1)+1) : (3*i);
        J_nodes(idx, idx) = J_blocks(:, :, i);
    end

    % Place in full-sized arrays
    ndof = numel(q);
    F_full = zeros(ndof, 1);
    J_full = zeros(ndof, ndof);

    F_full(1:softRobot.n_nodes*3) = F_nodes;
    J_full(1:softRobot.n_nodes*3, 1:softRobot.n_nodes*3) = J_nodes;

end
