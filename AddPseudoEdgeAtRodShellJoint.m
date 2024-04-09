function  [] = AddPseudoEdgeAtRodShellJoint(e0, f0, e0_sign, Nodes, edges, face_nodes_shell, MultiRod)

n_edges = MultiRod.n_edges;
n_edges_rod = MultiRod.n_edges_rod;
n_nodes = MultiRod.n_nodes;

% given edge e0 and face f0;
joint_edge_nodes = edges(e0, :);
joint_face_nodes = face_nodes_shell(f0, :);

% find node common to both structs above n0

for i=1:3
    if(joint_face_nodes(i) == joint_edge_nodes(1))
        n0 = joint_edge_nodes(1);
        ne = joint_edge_nodes(2);
        break;
    elseif(joint_face_nodes(i) == joint_edge_nodes(2))
        n0 = joint_edge_nodes(2);
        ne = joint_edge_nodes(1);
        break;
    else
        fprintf("input error: the rod and shell at the joint don't have any node in common");
    end
end

if(i==1) 
    n1 = joint_face_nodes(2);
    n2 = joint_face_nodes(3);
elseif(i==2) 
    n1 = joint_face_nodes(1);
    n2 = joint_face_nodes(3);
else 
    n1 = joint_face_nodes(1);
    n2 = joint_face_nodes(2);
end

% compute face normal and edge vectors
face_normal = cross((Nodes(joint_face_nodes(2),:)-Nodes(joint_face_nodes(1),:)), ...
    (Nodes(joint_face_nodes(3),:)-Nodes(joint_face_nodes(1),:)));

face_n = face_normal/norm(face_normal);

e0_vec = Nodes(n0,:) - Nodes(ne,:);

% project e0_vec on the triangle face plane
e = e0_vec - dot(face_n, e0_vec).* face_n;

% find the intersection between this projected edge e and the face's edge
% between the non-common nodes
f = Nodes(n1,:) - Nodes(n2,:);
g = Nodes(n1,:) - Nodes(n0,:);

l = norm(cross(f,g))/ norm(cross(f, e)) .* e;

if(dot(cross(f,g), cross(f, e)) > 0)
    M = n0 + l;
elseif(dot(cross(f,g), cross(f, e)) < 0)
    M = n0 - l;
else 
    fprintf("input shell mesh error: unable to draw a triangle at the joint face")
end

pseudo_e = [n0, n_nodes+1];
pseudo_e_vec = M - n0;
pseudo_bend_sp = [ne, e0, n0, n_edges_rod+1, n_nodes+1];

pseudo_bend_sp_sgn = [e0_sign, 1];

j = n_edges;
while j > n_edges_rod
    edges(j+1,:) = edges(j,:);
    j=j-1;
end
edges(n_edges_rod+1,:) = pseudo_e;

%%
n_edges = n_edges+1;
n_nodes = n_nodes + 1;
n_edges_rod = n_edges_rod+1;
% n_bend_twist = n_bend_twist+1;
Nodes(n_nodes,:) = M;

MultiRod.n_edges = n_edges;
MultiRod.n_nodes = n_nodes;
MultiRod.n_edges_rod = n_edges_rod;
% MultiRod.n_bend_twist = n_bend_twist;

MultiRod.Nodes = Nodes;
MultiRod.edges = edges;

MultiRod.voronoiRefLen(n_nodes) = 0.5*norm(pseudo_e_vec);

% stretch_springs(n_elStretchRod) = CreateStretchSpring(stretch_springs(n_elStretchRod), norm(pseudo_e_vec), pseudo_e);
% 
% bend_twist_springs(n_elBendRod) = CreateBendTwistSpring(bend_twist_springs(n_elBendRod), ...
%         pseudo_bend_sp, pseudo_bend_sp_sgn, MultiRod.voronoiRefLen, [0 0], 0, [MultiRod.EI, MultiRod.GJ]);

MultiRod.a1(n_edges_rod) = face_n;
MultiRod.a2(n_edges_rod) = cross(pseudo_e_vec/norm(pseudo_e_vec), face_n);




