function [Fc, Jc, Ffr, Jfr] = computeIMCContactAndFriction(q, q0, C, delta, contact_len, scale, k_c, mu_k, dt, vel_tol, n_dof, use_hess, friction_present)
% % Inputs
%   C: edge_combos which can potentially come in contact: Candidate set
% % Outputs
%   Fc: contact force
% -----------------------------------
assert(size(C, 2) == 4);
num_inputs = size(C, 1);
edge_combo_input = zeros(1,12);
ind = zeros(1,12);
Fc = zeros(n_dof,1);
Jc = zeros(n_dof,n_dof);
Ffr = zeros(n_dof,1);
Jfr = zeros(n_dof,n_dof);

K1 = 15*contact_len/(2*delta);
contact_lim   = scale*(contact_len + delta);
numerical_lim = scale*(contact_len- delta);

for i = 1:num_inputs
    [dist, constraint_type, edge_combo_idx_updated] = lumelskyMinDist(q, C(i,:), scale);
    for j = 1:4
        ind (3*j-2:3*j) = mapNodetoDOF(edge_combo_idx_updated(j));
        edge_combo_input(3*j-2:3*j) = q(ind (3*j-2:3*j));
    end
%% Contact
    % input
    input = [edge_combo_input.*scale, dist, contact_len/2*scale, K1];

    if ( dist<=numerical_lim )
        % if Δ <= 2h - δ: penetration
        if (constraint_type=="PointToPoint")
            gradEc = grad_E_pen_p2p(input);

            if(use_hess) 
                hessEc = hess_E_pen_p2p(input);
            else
                hessEc = zeros(12,12);
            end

        elseif (constraint_type=="PointToEdge")
            gradEc = grad_E_pen_p2e(input);

            if(use_hess) 
                hessEc = hess_E_pen_p2e(input);
            else
                hessEc = zeros(12,12);
            end

        elseif (constraint_type=="EdgeToEdge")
            gradEc = grad_E_pen_e2e(input);

            if(use_hess) 
                hessEc = hess_E_pen_e2e(input);
            else
                hessEc = zeros(12,12);
            end
        end

    elseif ( dist > numerical_lim && dist < contact_lim )
        % if (2h - δ) < Δ < (2h + δ): contact zone but no penetration
        if (constraint_type=="PointToPoint")
            gradEc = grad_E_con_p2p(input);
            
            if(use_hess) 
                hessEc = hess_E_con_p2p(input);
            else
                hessEc = zeros(12,12);
            end

        elseif (constraint_type=="PointToEdge")
            gradEc = grad_E_con_p2e(input);

            if(use_hess) 
                hessEc = hess_E_con_p2e(input);
            else
                hessEc = zeros(12,12);
            end

        elseif (constraint_type=="EdgeToEdge")
            gradEc = grad_E_con_e2e(input);

            if(use_hess) 
                hessEc = hess_E_con_e2e(input);
            else
                hessEc = zeros(12,12);
            end
        end
    else
        gradEc = zeros(1,12);
        hessEc = zeros(12,12);
    end
    fc = (scale * k_c).*gradEc;
    jc = (scale^2 * k_c).*hessEc;

    Fc(ind) = Fc(ind) - fc';
    Jc(ind,ind) = Jc(ind,ind) - jc;

    %% Friction
    if(friction_present)
        for j = 1:4
            edge_combo_input0(3*j-2:3*j) = q0(ind (3*j-2:3*j));
        end
    
        data = [edge_combo_input, edge_combo_input0];
        if (find(gradEc)) % only compute friction force if there is non-zero contact force
            [friction_type,ffr] = compute_friction(data, fc, mu_k, dt, vel_tol);
            Ffr(ind) = Ffr(ind) - ffr';
    
            if(use_hess)
                if(friction_type=="ZeroVel")
                    jfr = zeros(12,12);
                else
                    jfr = computeFrictionJacobian(data, fc, jc, mu_k, dt, vel_tol, friction_type, constraint_type);
                    
                end
                Jfr(ind,ind) = Jfr(ind,ind) - jfr;
            end
        end
    end

end


