function [q, a1, a2] = computePseudoEdgeParams(q, pseudo_edges, a1, a2)

n_pseudo_edges = size(pseudo_edges);

for i = 1: n_pseudo_edges
    joint_face_nodes = pseudo_edges(i).joint_face_nodes;
    n0 = pseudo_edges(i).n0;
    ne = pseudo_edges(i).ne;
    n1 = pseudo_edges(i).n1;
    n2 = pseudo_edges(i).n2;

    pseudo_n_ind = pseudo_edges(i).pseudo_n_ind;
    
    % compute face normal and edge vectors
    face_normal = cross( ( q(mapNodetoDOF(joint_face_nodes(2)))- q(mapNodetoDOF(joint_face_nodes(1))) ), ...
    ( q(mapNodetoDOF(joint_face_nodes(3)))- q(mapNodetoDOF(joint_face_nodes(1))) ) );

    face_n = face_normal/norm(face_normal);

    e0_vec = q(mapNodetoDOF(n0)) - q(mapNodetoDOF(ne));

    % project e0_vec on the triangle face plane
    e = e0_vec - dot(face_n, e0_vec).* face_n;
    
    % find the intersection between this projected edge e and the face's edge
    % between the non-common nodes
    f = q(mapNodetoDOF(n1)) - q(mapNodetoDOF(n2));
    g = q(mapNodetoDOF(n1)) - q(mapNodetoDOF(n0));

    l = norm(cross(f,g))/ norm(cross(f, e)) .* e;

    assert(abs(dot(cross(f,g), cross(f, e)))> 10^-8, ...
        "shell mesh error: unable to draw a triangle at the joint face");

    if(dot(cross(f,g), cross(f, e)) > 0)
        M = n0 + l;
    else
        M = n0 - l;
    end

    pseudo_e_vec = M - n0;

    % modify the value of the DOF vector location for pseudo node as M
    q(mapNodetoDOF(pseudo_n_ind)) = M;

    % modify the value of the reference frame a1 a2
    a1(i,:) = face_n;
    a2(i,:) = cross(pseudo_e_vec/norm(pseudo_e_vec), face_n);
end


