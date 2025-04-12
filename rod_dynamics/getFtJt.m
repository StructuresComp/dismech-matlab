function [Ft, Jt, bend_twist_springs] = getFtJt(MultiRod, bend_twist_springs, q, refTwist, sim_params)

n_DOF = MultiRod.n_DOF;
n_nodes = MultiRod.n_nodes;
n_twist = numel(bend_twist_springs);
undef_refTwist = MultiRod.undef_refTwist;

Ft = zeros(n_DOF,1);
Jt = zeros(n_DOF);

% refTwist-undef_refTwist

for c = 1:n_twist
    n0 = bend_twist_springs(c).nodes_ind(1);
    n1 = bend_twist_springs(c).nodes_ind(2);
    n2 = bend_twist_springs(c).nodes_ind(3);
    e0 = bend_twist_springs(c).edges_ind(1);
    e1 = bend_twist_springs(c).edges_ind(2);

    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    node2p = q(mapNodetoDOF(n2))';

    theta_e = bend_twist_springs(c).sgn(1) * q(mapEdgetoDOF(e0, n_nodes));
    theta_f = bend_twist_springs(c).sgn(2) * q(mapEdgetoDOF(e1, n_nodes));

    ind = bend_twist_springs(c).ind; % Size 11

    if(isfield(sim_params, 'bergou_DER') )
        if(sim_params.bergou_DER)
            % Bergou
            [dF, dJ] = ...
                gradEt_hessEt_struct_new(n_DOF, ind, node0p, node1p, node2p, ...
                theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));
        else
            % Panetta
            [dF, dJ] = ...
                gradEt_hessEt_panetta(n_DOF, ind, node0p, node1p, node2p, ...
                theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));
        end
    else
        % Panetta
        [dF, dJ] = ...
            gradEt_hessEt_panetta(n_DOF, ind, node0p, node1p, node2p, ...
            theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));
    end

    %% change sign of forces if the edges were flipped for alignment earlier
    if bend_twist_springs(c).sgn(1) ~= 1
        dF(mapEdgetoDOF(e0, n_nodes)) = - dF(mapEdgetoDOF(e0, n_nodes));
        dJ(mapEdgetoDOF(e0, n_nodes), :) = - dJ(mapEdgetoDOF(e0, n_nodes), :);
        dJ(:, mapEdgetoDOF(e0, n_nodes)) = - dJ(:, mapEdgetoDOF(e0, n_nodes));
    end
    
    if bend_twist_springs(c).sgn(2) ~= 1
        dF(mapEdgetoDOF(e1, n_nodes)) = - dF(mapEdgetoDOF(e1, n_nodes));
        dJ(mapEdgetoDOF(e1, n_nodes), :) = - dJ(mapEdgetoDOF(e1, n_nodes), :);
        dJ(:, mapEdgetoDOF(e1, n_nodes)) = - dJ(:, mapEdgetoDOF(e1, n_nodes));
    end

    Ft(ind) = Ft(ind) - dF(ind);
    Jt(ind, ind) = Jt(ind, ind) - dJ(ind, ind);

    %% update spring forces in the spring structs
    bend_twist_springs(c).dFt = dF(ind);
    bend_twist_springs(c).dJt = dJ(ind, ind);

end
end
