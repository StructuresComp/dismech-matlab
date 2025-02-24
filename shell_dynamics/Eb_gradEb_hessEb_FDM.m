function [E_with_stiff, gradE_with_stiff, hessE_FDM, t_i, t_j, t_k, c_i, c_j, c_k] = ...
    Eb_gradEb_hessEb_FDM(stiff, nu, pi, pj, pk, xi_i, xi_j, xi_k, s_i, s_j, s_k, ...
    tau_i0, tau_j0, tau_k0, A, ls, init_ts, init_cs, init_fs, init_xi, ...
    optional_t_i, optional_t_j, optional_t_k, optional_c_i, optional_c_j, optional_c_k)

    % Compute energy and gradient using the original function
%     [E_with_stiff, gradE_with_stiff, ~, t_i, t_j, t_k, c_i, c_j, c_k] = ...
%         Eb_gradEb_hessEb_shell_midedge(stiff, nu, pi, pj, pk, xi_i, xi_j, xi_k, s_i, s_j, s_k, ...
%         tau_i0, tau_j0, tau_k0, A, ls, init_ts, init_cs, init_fs, init_xi, ...
%         optional_t_i, optional_t_j, optional_t_k, optional_c_i, optional_c_j, optional_c_k);
   [E_with_stiff, gradE_with_stiff, ~, t_i, t_j, t_k, c_i, c_j, c_k] = ...
        Eb_gradEb_hessEb_shell_midedge(stiff, nu, pi, pj, pk, xi_i, xi_j, xi_k, s_i, s_j, s_k, ...
        tau_i0, tau_j0, tau_k0, A, ls, init_ts, init_cs, init_fs, init_xi);

    % Combine input variables into a single vector for numerical differentiation
    x = [pi(:); pj(:); pk(:); xi_i; xi_j; xi_k];
    n = length(x);
    hessE_FDM = zeros(n, n);
    epsilon = 1e-6;

    % Compute Hessian using finite difference method
    for i = 1:n
        x_perturb_pos = x;
        x_perturb_neg = x;
        x_perturb_pos(i) = x_perturb_pos(i) + epsilon;
        x_perturb_neg(i) = x_perturb_neg(i) - epsilon;
        
        % Extract perturbed values
        [pi_p, pj_p, pk_p, xi_i_p, xi_j_p, xi_k_p] = extract_variables(x_perturb_pos);
        [pi_n, pj_n, pk_n, xi_i_n, xi_j_n, xi_k_n] = extract_variables(x_perturb_neg);
        
        % Compute perturbed gradients
%         [~, grad_pos, ~, ~, ~, ~, ~, ~, ~] = ...
%             Eb_gradEb_hessEb_shell_midedge(stiff, nu, pi_p, pj_p, pk_p, xi_i_p, xi_j_p, xi_k_p, s_i, s_j, s_k, ...
%             tau_i0, tau_j0, tau_k0, A, ls, init_ts, init_cs, init_fs, init_xi, ...
%             optional_t_i, optional_t_j, optional_t_k, optional_c_i, optional_c_j, optional_c_k);
%         
%         [~, grad_neg, ~, ~, ~, ~, ~, ~, ~] = ...
%             Eb_gradEb_hessEb_shell_midedge(stiff, nu, pi_n, pj_n, pk_n, xi_i_n, xi_j_n, xi_k_n, s_i, s_j, s_k, ...
%             tau_i0, tau_j0, tau_k0, A, ls, init_ts, init_cs, init_fs, init_xi, ...
%             optional_t_i, optional_t_j, optional_t_k, optional_c_i, optional_c_j, optional_c_k);
%        
        [~, grad_pos, ~, ~, ~, ~, ~, ~, ~] = ...
            Eb_gradEb_hessEb_shell_midedge(stiff, nu, pi_p, pj_p, pk_p, xi_i_p, xi_j_p, xi_k_p, s_i, s_j, s_k, ...
            tau_i0, tau_j0, tau_k0, A, ls, init_ts, init_cs, init_fs, init_xi);
        
        [~, grad_neg, ~, ~, ~, ~, ~, ~, ~] = ...
            Eb_gradEb_hessEb_shell_midedge(stiff, nu, pi_n, pj_n, pk_n, xi_i_n, xi_j_n, xi_k_n, s_i, s_j, s_k, ...
            tau_i0, tau_j0, tau_k0, A, ls, init_ts, init_cs, init_fs, init_xi);
        
        % Compute second derivative approximation
        hessE_FDM(:, i) = (grad_pos - grad_neg) / (2 * epsilon);
    end
end

function [pi, pj, pk, xi_i, xi_j, xi_k] = extract_variables(x)
    % Extracts variables from input vector x
    pi = x(1:3);
    pj = x(4:6);
    pk = x(7:9);
    xi_i = x(10);
    xi_j = x(11);
    xi_k = x(12);
end
