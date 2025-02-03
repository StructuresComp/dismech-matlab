function [Fb, Jb, Ft, Jt, Jb_FDM, Jt_FDM, bend_twist_springs] = getFbJb_FtJt_and_FDM(MultiRod, bend_twist_springs, q, m1, m2, refTwist, sim_params)

global bug 

n_DOF = MultiRod.n_DOF;
n_nodes = MultiRod.n_nodes;
n_bend = numel(bend_twist_springs);

Fb = zeros(n_DOF,1);
Jb = zeros(n_DOF, n_DOF);

Ft = zeros(n_DOF,1);
Jt = zeros(n_DOF, n_DOF);

Jb_FDM = zeros(n_DOF, n_DOF);
Jt_FDM = zeros(n_DOF, n_DOF);

a1 = MultiRod.a1;
undef_refTwist = MultiRod.undef_refTwist;
change = 1e-8;

for c = 1:n_bend
    n0 = bend_twist_springs(c).nodes_ind(1);
    n1 = bend_twist_springs(c).nodes_ind(2);
    n2 = bend_twist_springs(c).nodes_ind(3);
    e0 = bend_twist_springs(c).edges_ind(1);
    e1 = bend_twist_springs(c).edges_ind(2);

    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    node2p = q(mapNodetoDOF(n2))';

    m1e = m1(e0,:);
    m2e = bend_twist_springs(c).sgn(1) * m2(e0,:);
    m1f = m1(e1,:);
    m2f = bend_twist_springs(c).sgn(2) * m2(e1,:);

    theta_e = bend_twist_springs(c).sgn(1) * q(mapEdgetoDOF(e0, n_nodes));
    theta_f = bend_twist_springs(c).sgn(2) * q(mapEdgetoDOF(e1, n_nodes));

%%
    ind = bend_twist_springs(c).ind; % Size 11

    if(isfield(sim_params, 'bergou_DER') )
        if(sim_params.bergou_DER)
            % Bergou
            [dFb, dJb] = ...
                gradEb_hessEb_struct(n_DOF, ind, node0p, node1p, node2p, ...
                m1e, m2e, m1f, m2f, bend_twist_springs(c));
            [dFt, dJt] = ...
                gradEt_hessEt_struct_new(n_DOF, ind, node0p, node1p, node2p, ...
                theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));

        else
            % Panetta
            [dFb, dJb] = ...
                gradEb_hessEb_panetta(n_DOF, ind, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
            [dFt, dJt] = ...
                gradEt_hessEt_panetta(n_DOF, ind, node0p, node1p, node2p, ...
                theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));
        end
    else
        % Panetta
        [dFb, dJb] = ...
            gradEb_hessEb_panetta(n_DOF, ind, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
        [dFt, dJt] = ...
                gradEt_hessEt_panetta(n_DOF, ind, node0p, node1p, node2p, ...
                theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));
    end

    % if(isfield(sim_params, 'FDM'))
    %     if(sim_params.FDM)
            Fb_ = ...
                gradEb(n_DOF, ind, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
            Ft_ = gradEt_new(n_DOF, ind, node0p, node1p, node2p, ...
        theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));

            % Fb_ = -getFb(MultiRod, bend_twist_springs, q, m1, m2);
            % Ft_ = -getFt(MultiRod, bend_twist_springs, q, refTwist); % twisting

            for i=1:size(ind,1)
                q_change = q;
                q_change(ind(i)) = q(ind(i)) + change;

                % Compute time parallel reference frame
                [a1_change, a2_change] = computeTimeParallel(MultiRod, a1, q, q_change); % q or q0 in second last argument

                % Compute reference twist
                tangent_change = computeTangent(MultiRod, q_change);
                refTwist_change = computeRefTwist_bend_twist_spring(bend_twist_springs, a1_change, tangent_change, refTwist);

                % Compute material frame
                theta_change = q_change(3*n_nodes + 1 : 3*n_nodes + MultiRod.n_edges_dof);
                [m1_change, m2_change] = computeMaterialDirectors(a1_change,a2_change,theta_change);


                node0p_change = q_change(mapNodetoDOF(n0))';
                node1p_change = q_change(mapNodetoDOF(n1))';
                node2p_change = q_change(mapNodetoDOF(n2))';

                m1e = m1_change(e0,:);
                m2e = bend_twist_springs(c).sgn(1) * m2_change(e0,:);
                m1f = m1_change(e1,:);
                m2f = bend_twist_springs(c).sgn(2) * m2_change(e1,:);

                theta_e = bend_twist_springs(c).sgn(1) * q_change(mapEdgetoDOF(e0, n_nodes));
                theta_f = bend_twist_springs(c).sgn(2) * q_change(mapEdgetoDOF(e1, n_nodes));

                
                % changes in the gradient
                % Fb_change = -getFb(MultiRod, bend_twist_springs, q_change, m1_change, m2_change);
                % dJb_FDM(i,:) = (Fb_change - Fb_) .* (1/change);
                % 
                % Ft_change = -getFt(MultiRod, bend_twist_springs, q_change, refTwist_change); % twisting
                % dJt_FDM(i,:) = (Ft_change - Ft_) .* (1/change);

                Fb_change = gradEb(n_DOF, ind, node0p_change, node1p_change, node2p_change, m1e, m2e, m1f, m2f, bend_twist_springs(c));

                dJb_FDM(i,:) = (Fb_change - Fb_) .* (1/change);

                Ft_change = gradEt_new(n_DOF, ind, node0p_change, node1p_change, node2p_change, ...
                    theta_e, theta_f, refTwist_change(c), bend_twist_springs(c), undef_refTwist(c));
                dJt_FDM(i,:) = (Ft_change - Ft_) .* (1/change);
            end
        % end
    %     dJb_FDM(:, ind)
    %     % dJt_FDM(:, ind)
    % % end
    % dJb(ind, ind)
    % % dJt(ind, ind)
% 
%             figure(100)
% subplot(2,1,1)
% plot( reshape(dJb(ind, ind), [121,1]), 'ro');
% hold on
% plot( reshape(dJb_FDM(:, ind), [121,1]), 'b^');
% hold off
% legend('Analytical', 'Finite Difference');
% xlabel('Index');
% ylabel('Hessian');
% title('Bending hessian')
% 
% subplot(2,1,2)
% plot( reshape(dJt(ind, ind), [121,1]), 'ro');
% hold on
% plot( reshape(dJt_FDM(:, ind), [121,1]), 'b^');
% hold off
% legend('Analytical', 'Finite Difference');
% xlabel('Index');
% ylabel('Hessian');
% title('Bending hessian')
% 

    %% change sign of forces if the edges were flipped for alignment earlier
    if bend_twist_springs(c).sgn(1) ~= 1
        dFb(mapEdgetoDOF(e0, n_nodes)) = - dFb(mapEdgetoDOF(e0, n_nodes));
        dJb(mapEdgetoDOF(e0, n_nodes), :) = - dJb(mapEdgetoDOF(e0, n_nodes), :);
        dJb(:, mapEdgetoDOF(e0, n_nodes)) = - dJb(:, mapEdgetoDOF(e0, n_nodes));
    end
    
    if bend_twist_springs(c).sgn(2) ~= 1
        dFb(mapEdgetoDOF(e1, n_nodes)) = - dFb(mapEdgetoDOF(e1, n_nodes));
        dJb(mapEdgetoDOF(e1, n_nodes), :) = - dJb(mapEdgetoDOF(e1, n_nodes), :);
        dJb(:, mapEdgetoDOF(e1, n_nodes)) = - dJb(:, mapEdgetoDOF(e1, n_nodes));
    end

    Fb(ind) = Fb(ind) - dFb(ind);
    Jb(ind, ind) = Jb(ind, ind) - dJb(ind, ind);
    Ft(ind) = Ft(ind) - dFt(ind);
    Jt(ind, ind) = Jt(ind, ind) - dJt(ind, ind);

    
    Jb_FDM(ind, ind) = Jb_FDM(ind, ind) - dJb_FDM(:, ind);
    Jt_FDM(ind, ind) = Jt_FDM(ind, ind) - dJt_FDM(:, ind);


    if(isnan(sum(Fb)))
        Fb
    end
    if(isnan(sum(dJb(ind, ind),"all")))
        Jb
    end

    %% to debug
    for i=1:numel(dFb)
        if (dFb(i)~= 0 && ~find(ind==i))
            fprintf("Bug: dF getting changed at wrong indices")
            bug=1;
        end
        for j=1:numel(dFb)
            if (dJb(i,j)~= 0 && (~find(ind==i) || ~find(ind==j)))
                fprintf("Bug: dJ getting changed at wrong indices")
                bug=1;
            end
        end

    end
    %% update spring forces in the spring structs
    bend_twist_springs(c).dFb = dFb(ind);
    bend_twist_springs(c).dJb = dJb(ind, ind);
    bend_twist_springs(c).dFt = dFt(ind);
    bend_twist_springs(c).dJt = dJt(ind, ind);
end
end
