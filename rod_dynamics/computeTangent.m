function tangent = computeTangent(MultiRod, q)
n_edges_dof = MultiRod.n_edges_dof;
edges = MultiRod.Edges;
tangent = zeros(n_edges_dof,3);

for i=1:n_edges_dof
    n0 = edges(i,1);
    n1 = edges(i,2);
    node0_pos = q(mapNodetoDOF(n0));
    node1_pos = q(mapNodetoDOF(n1));
    de = (node1_pos - node0_pos)' ; 
    tangent_vec = de/norm(de);
    % make sure tangent is unit vector remove small non-zero terms if any
    tangent_vec(abs(tangent_vec)<1e-10) = 0;

    tangent(i,:) = tangent_vec;
end

end

