function [tau_0] = updatePreComp_without_sign(q,MultiRod)
Face_Nodes = MultiRod.face_nodes_shell';
face_edges = MultiRod.face_edges;
EdgeIsBetween = MultiRod.Edges';
n_edges = MultiRod.n_edges;
n_faces = MultiRod.n_faces;

e = zeros(3, n_edges);
edge_common_to = zeros(n_edges,1);
n_avg = zeros(3,n_edges);
tau_0 = zeros(3,n_edges);

for c = 1:n_faces
    node1_number = Face_Nodes(1,c);
    node2_number = Face_Nodes(2,c);
    node3_number = Face_Nodes(3,c);
    node1_position = q(3*node1_number-2:3*node1_number);
    node2_position = q(3*node2_number-2:3*node2_number);
    node3_position = q(3*node3_number-2:3*node3_number);
    
    % face normal calculation:
    face_normal = cross(([node2_position]-[node1_position]),([node3_position]-[node1_position]));
    face_unit_normal = face_normal .* 1/norm(face_normal);

    % face edge map
    edge1_idx = face_edges(c,1);
    edge2_idx = face_edges(c,2);
    edge3_idx = face_edges(c,3);

    edge_common_to(edge1_idx) = edge_common_to(edge1_idx)+1;
    edge_common_to(edge2_idx) = edge_common_to(edge2_idx)+1;
    edge_common_to(edge3_idx) = edge_common_to(edge3_idx)+1;

    n_avg(:,edge1_idx) = n_avg(:,edge1_idx) + face_unit_normal;
    n_avg(:,edge1_idx) = n_avg(:,edge1_idx)/ norm(n_avg(:,edge1_idx));

    n_avg(:,edge2_idx) = n_avg(:,edge2_idx) + face_unit_normal;
    n_avg(:,edge2_idx) = n_avg(:,edge2_idx)/ norm(n_avg(:,edge2_idx));

    n_avg(:,edge3_idx) = n_avg(:,edge3_idx) + face_unit_normal;
    n_avg(:,edge3_idx) = n_avg(:,edge3_idx)/ norm(n_avg(:,edge3_idx));

    % ensure that edge is common to only 2 triangle faces (to avoid bugs)
    assert(edge_common_to(edge1_idx)<3, "edge is common to more than 2 faces!");
    assert(edge_common_to(edge2_idx)<3, "edge is common to more than 2 faces!");
    assert(edge_common_to(edge3_idx)<3, "edge is common to more than 2 faces!");

end

for i=1:n_edges
    e(:,i) = q(3*EdgeIsBetween(2,i)-2:3*EdgeIsBetween(2,i)) - q(3*EdgeIsBetween(1,i)-2:3*EdgeIsBetween(1,i));
    tau_0(:,i) = cross(e(:,i), n_avg(:,i));
    tau_0(:,i) = tau_0(:,i)/norm(tau_0(:,i));
end
