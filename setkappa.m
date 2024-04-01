function bend_twist_springs = setkappa(MultiRod, bend_twist_springs)

q = MultiRod.q;
m1 = MultiRod.m1;
m2 = MultiRod.m2;

n_bend = numel(bend_twist_springs);

for i=1:n_bend
    
    edge1_index = bend_twist_springs(i).edges_ind (1);
    edge2_index = bend_twist_springs(i).edges_ind (2);
    node1_index = bend_twist_springs(i).nodes_ind (1);
    node2_index = bend_twist_springs(i).nodes_ind (2);
    node3_index = bend_twist_springs(i).nodes_ind (3);

    node1_loc = q (mapNodetoDOF(node1_index));
    node2_loc = q (mapNodetoDOF(node2_index));
    node3_loc = q (mapNodetoDOF(node3_index));

    m1e = m1(edge1_index,:);
    m2e = bend_twist_springs(i).sgn(1) * m2(edge1_index,:);
    m1f = m1(edge2_index,:);
    m2f = bend_twist_springs(i).sgn(2) * m2(edge2_index,:);

    kappaL = computekappa(node1_loc, node2_loc, node3_loc, m1e, m2e, m1f, m2f );

    bend_twist_springs(i).kappaBar = kappaL;
end

end

function kappa_L = computekappa(node0, node1, node2, m1e, m2e, m1f, m2f )
%
% Inputs:
% node0: 1x3 vector - position of the node prior to the "turning" node
% node1: 1x3 vector - position of the "turning" node
% node2: 1x3 vector - position of the node after the "turning" node
%
% m1e: 1x3 vector - material director 1 of the edge prior to turning
% m2e: 1x3 vector - material director 2 of the edge prior to turning
% m1f: 1x3 vector - material director 1 of the edge after turning
% m2f: 1x3 vector - material director 2 of the edge after turning
%
% Outputs:
% kappa: 1x2 vector - curvature at the turning node

t0 = (node1-node0) / norm(node1-node0);
t1 = (node2-node1) / norm(node2-node1);
kb = 2.0 * cross(t0, t1) / (1.0 + dot(t0, t1));

kappa_L = zeros(1, 2);
kappa1 = 0.5 * dot( kb, m2e + m2f);
kappa2 = -0.5 * dot( kb, m1e + m1f);
kappa_L(1) = kappa1;
kappa_L(2) = kappa2;

end
