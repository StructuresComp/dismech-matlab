function tangent_vec = computeTangent(MultiRod, q)
n_edges_rod = MultiRod.n_edges_rod;
edges = MultiRod.Edges;
tangent_vec = zeros(n_edges_rod,3);

for i=1:n_edges_rod
    n0 = edges(i,1);
    n1 = edges(i,2);
    node0_pos = q(mapNodetoDOF(n0));
    node1_pos = q(mapNodetoDOF(n1));
    de = (node1_pos - node0_pos)' ; 
    tangent_vec(i,:) = de/norm(de);
end

end

