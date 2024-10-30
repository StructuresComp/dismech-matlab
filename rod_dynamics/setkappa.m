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