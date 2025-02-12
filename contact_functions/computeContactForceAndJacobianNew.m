function [Fc, Jc] = computeContactForceAndJacobianNew(q, C, delta, h, scale, n_dof, use_hess)
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

K1 = 15*h/delta;
contact_lim   = scale*(2*h + delta);
numerical_lim = scale*(2*h - delta);

for i = 1:num_inputs
    [dist, constraint_type, edge_combo_idx_updated] = lumelskyMinDist(q, C(i,:), scale);
    for j = 1:4
        ind (3*j-2:3*j) = mapNodetoDOF(edge_combo_idx_updated(j));
        edge_combo_input(3*j-2:3*j) = q(ind (3*j-2:3*j)).*scale;
    end

    % input
    input = [edge_combo_input, dist, h*scale, K1];

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
Fc(ind) = Fc(ind) - gradEc';
Jc(ind,ind) = Jc(ind,ind) - hessEc;

% if(find(Fc))
%     Fc
% end
end


