function MultiRod = makeMultiRod (MultiRod, Nodes, face_nodes, twist_angles, Edges, r0, h, rho, Y_rod, nu_rod, Y_shell, nu_shell)

% Inputs:
% nodes: n_nodesx3 vector of node initial (x,y,z) positions
% twist_angles: n_edgesx1 vector of initial angle theta for each edge
% edges: n_edgesx2 vector showing the indices of the 2 nodes that make up
% each edge
% rho: density of the rod material
% Y: young's modulus of the rod material
% nu: Poisson's ratio of the rod material

% Output:
% MultiRod struct that has the following fields:
% MultiRod.n_nodes
% MultiRod.n_edges
% MultiRod.n_DOF
% MultiRod.Nodes 
% MultiRod.Edges
% MultiRod.q0
% MultiRod.q
% MultiRod.refLen 
% MultiRod.voronoiRefLen
% MultiRod.MassMat
% MultiRod.W 
% MultiRod.EI
% MultiRod.EA
% MultiRod.GJ
% MultiRod.u

%% function starts here
global g

n_nodes = size(Nodes,1);
n_edges = size(Edges,1);
n_edges_rod = size(twist_angles,1);
if (n_edges_rod) 
    n_nodes_rod = n_edges_rod+1;
else
    n_nodes_rod = 0;
end
n_faces_shell = size(face_nodes,1);

n_DOF = 3*n_nodes + n_edges_rod;

%% DOF vector: first 3*n_nodes terms => x,y,z coordinates of all nodes; next
% n_edges terms => thetas of each edge for rod 
q0 = zeros(n_DOF,1);
q_nodes=reshape(Nodes',[numel(Nodes),1]);
q0(1:3*n_nodes) = q_nodes; 
q0(3*n_nodes + 1 : 3*n_nodes + n_edges_rod) = twist_angles;

% % or
% for i=1:n_nodes
%     ind = mapNodetoDOF(i);
%     q(ind) = Nodes(i,:)';
% end
% 
% for j=1:n_edges
%     ind = mapEdgetoDOF(j);
%     q(ind) = twist_angles(j);
% end

%% reference length (edge based quantity)

refLen = zeros(n_edges,1);
for c=1:n_edges
    node1_index = Edges(c,1);
    node2_index = Edges(c,2);
    refLen(c) = norm(Nodes(node2_index,:)-Nodes(node1_index,:));
end
%% voronoi reference length (node based quantity)

voronoiRefLen = zeros(n_nodes,1);
% voronoiRefLen = zeros(57,1);
for c=1:n_edges_rod   
    node1_index = Edges(c,1);
    node2_index = Edges(c,2);
    voronoiRefLen(node1_index) = voronoiRefLen(node1_index) + 0.5 * refLen(c);
    voronoiRefLen(node2_index) = voronoiRefLen(node2_index) + 0.5 * refLen(c);    
end

%% Compute Mass

m = zeros(n_DOF, 1);

% m = zeros(3*n_nodes,1);

for i=1:n_faces_shell
    
    node1ind = face_nodes(i,1);
    node2ind = face_nodes(i,2);
    node3ind = face_nodes(i,3);
    face_A = 0.5 * norm(cross( (Nodes(node2ind,:)-Nodes(node1ind,:)), (Nodes(node3ind,:) - Nodes(node2ind,:)) ) );
    Mface = rho * face_A * h;

    m(mapNodetoDOF(node1ind)) = m(mapNodetoDOF(node1ind)) + Mface/3.*ones(3,1);
    m(mapNodetoDOF(node2ind)) = m(mapNodetoDOF(node2ind)) + Mface/3.*ones(3,1);
    m(mapNodetoDOF(node3ind)) = m(mapNodetoDOF(node3ind)) + Mface/3.*ones(3,1);
end

for cNode=1:n_nodes
    dm = voronoiRefLen(cNode) * pi * r0^2 * rho;
    ind = mapNodetoDOF(cNode);
    m(ind) = m(ind) + dm*ones(3,1);
end

for cEdge=1:n_edges_rod
    dm = refLen(cEdge) * pi * r0^2 * rho;
    ind = mapEdgetoDOF(cEdge,n_nodes);
    m(ind) = dm/2 * r0^2; % I = 1/2 m r ^ 2
end
% for now to debug:
% m(1:n_nodes*3) = m(4,1); 
massMat = diag(m);

%% Weight
W = zeros(n_DOF, 1);
for cNode=1:n_nodes
        ind = mapNodetoDOF(cNode);
        W(ind) = m(ind) .* g;
end

%% Stiffnesses
G_rod = Y_rod/(2.0*(1.0+nu_rod)); % Shear modulus
EI = Y_rod * pi * r0^4/4; % Bending stiffness
GJ = G_rod * pi * r0^4/2; % Shearing stiffness
EA = Y_rod * pi * r0^2; % Stretching stiffness

% Shell
ks = sqrt(3)/2 * Y_shell * h *refLen(n_edges_rod+1:end).^3;
kb = 2/sqrt(3) * Y_shell *(h^3)/12;

%% assign values to the struct
MultiRod.n_nodes = n_nodes;
MultiRod.n_edges = n_edges;
MultiRod.n_edges_rod = n_edges_rod;
MultiRod.n_DOF = n_DOF;
MultiRod.Nodes = Nodes;
MultiRod.Edges = Edges;
MultiRod.face_nodes_shell = face_nodes;
MultiRod.q0 = q0; % to store initial value of DOF
MultiRod.q = q0; % DOF vector initialized
MultiRod.refLen = refLen;
MultiRod.voronoiRefLen = voronoiRefLen;
MultiRod.MassMat = massMat;
MultiRod.W = W;
MultiRod.EI = EI;
MultiRod.EA = EA;
MultiRod.GJ = GJ;
MultiRod.ks = ks;
MultiRod.kb = kb;
MultiRod.u = zeros(size(q0)); % to store velocities in the future
% storage vectors for reference frames
MultiRod.a1 = zeros(n_edges_rod,3);
MultiRod.a2 = zeros(n_edges_rod,3);
MultiRod.m1 = zeros(n_edges_rod,3);
MultiRod.m2 = zeros(n_edges_rod,3);

end
