function [Ft, Jt, bend_twist_springs] = getFtJt(MultiRod, bend_twist_springs, q, refTwist)
global bug 

n_DOF = MultiRod.n_DOF;
n_nodes = MultiRod.n_nodes;
n_twist = numel(bend_twist_springs);
undef_refTwist = MultiRod.undef_refTwist;

Ft = zeros(n_DOF,1);
Jt = zeros(n_DOF);

for c = 1:n_twist
    n0 = bend_twist_springs(c).nodes_ind(1);
    n1 = bend_twist_springs(c).nodes_ind(2);
    n2 = bend_twist_springs(c).nodes_ind(3);
    e0 = bend_twist_springs(c).edges_ind(1);
    e1 = bend_twist_springs(c).edges_ind(2);

    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    node2p = q(mapNodetoDOF(n2))';
%     theta_e = q(mapEdgetoDOF(e0, n_nodes));
%     theta_f = q(mapEdgetoDOF(e1, n_nodes));

    theta_e = bend_twist_springs(c).sgn(1) * q(mapEdgetoDOF(e0, n_nodes));
    theta_f = bend_twist_springs(c).sgn(2) * q(mapEdgetoDOF(e1, n_nodes));

    ind = bend_twist_springs(c).ind; % Size 11

    % [dF, dJ] = ...
    %     gradEt_hessEt_struct(n_DOF, ind, node0p, node1p, node2p, ...
    %     theta_e, theta_f, refTwist(c), bend_twist_springs(c));
[dF, dJ] = ...
        gradEt_hessEt_struct_new(n_DOF, ind, node0p, node1p, node2p, ...
        theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));

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

    %% to debug
    for i=1:numel(dF)
        if (dF(i)~= 0 && ~find(ind==i))
            fprintf("Bug: dF getting changed at wrong indices")
            bug=1;
        end
    for j=1:numel(dF)
        if (dJ(i,j)~= 0 && (~find(ind==i) || ~find(ind==j)))
            fprintf("Bug: dJ getting changed at wrong indices")
            bug=1;
        end
    end

    end
    %% update spring forces in the spring structs
    bend_twist_springs(c).dFt = dF(ind);
    bend_twist_springs(c).dJt = dJ(ind, ind);

end
end
