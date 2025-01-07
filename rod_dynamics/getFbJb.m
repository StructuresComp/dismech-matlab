function [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2, sim_params)

global bug 

n_DOF = MultiRod.n_DOF;
n_nodes = MultiRod.n_nodes;
n_bend = numel(bend_twist_springs);

Fb = zeros(n_DOF,1);
Jb = zeros(n_DOF);

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
%%
    ind = bend_twist_springs(c).ind; % Size 11

    if(isfield(sim_params, 'bergou_DER') )
        if(sim_params.bergou_DER)
            % Bergou
            [dF, dJ] = ...
                gradEb_hessEb_struct(n_DOF, ind, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
        else
            % Panetta
            [dF, dJ] = ...
                gradEb_hessEb_panetta(n_DOF, ind, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
        end
    else
        % Panetta
        [dF, dJ] = ...
            gradEb_hessEb_panetta(n_DOF, ind, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
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

    Fb(ind) = Fb(ind) - dF(ind);
    Jb(ind, ind) = Jb(ind, ind) - dJ(ind, ind);
    if(isnan(sum(Fb)))
        Fb
    end
    if(isnan(sum(dJ(ind, ind),"all")))
        Jb
    end

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
    bend_twist_springs(c).dFb = dF(ind);
    bend_twist_springs(c).dJb = dJ(ind, ind);
end
end
