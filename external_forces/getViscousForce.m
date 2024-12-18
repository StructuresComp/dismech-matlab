function [Fv, Jv] = getViscousForce(q,q0,dt,eta, MultiRod)
u = (q-q0)/dt ;
Fv = zeros(MultiRod.n_DOF,1);
Jv = zeros(MultiRod.n_DOF,MultiRod.n_DOF);
l_edges = MultiRod.refLen;

for c=1:MultiRod.n_edges 
    node1ind = MultiRod.Edges(c,1);
    node2ind = MultiRod.Edges(c,2);
    idx = [mapNodetoDOF(node1ind); mapNodetoDOF(node2ind)];
    edge_vec = q(mapNodetoDOF(node2ind))-q(mapNodetoDOF(node1ind));
    edge_len =  l_edges(c);
    edge_hat = edge_vec/norm(edge_vec);

    u1 = u(mapNodetoDOF(node1ind));
    u2 = u(mapNodetoDOF(node2ind));

    Fv(idx(1:3)) = Fv(idx(1:3)) + (eta*edge_len/2).* ...
        -( u1 - dot(u1, edge_hat).*edge_hat );
    Fv(idx(4:6)) = Fv(idx(4:6)) + (eta*edge_len/2).* ...
        -( u2 - dot(u2, edge_hat).*edge_hat );

    Jv(idx(1:3),idx(1:3)) = Jv(idx(1:3),idx(1:3)) - (eta*edge_len/2).*(eye(3)/dt + edge_hat*(u1 - edge_hat/dt)' - dot(u1,edge_hat)/norm(edge_vec)^3 *(edge_vec*edge_vec' - norm(edge_vec)^2*eye(3)));
    Jv(idx(4:6),idx(4:6)) = Jv(idx(4:6),idx(4:6)) - (eta*edge_len/2).*(eye(3)/dt + edge_hat*(u2 - edge_hat/dt)' - dot(u2,edge_hat)/norm(edge_vec)^3 *(edge_vec*edge_vec' - norm(edge_vec)^2*eye(3)));
    Jv(idx(1:3),idx(4:6)) = Jv(idx(1:3),idx(4:6)) + (eta*edge_len/2).*(edge_hat*u1' + u1*edge_hat'*eye(3))*(norm(edge_vec)^2*eye(3) - edge_vec*edge_vec')/norm(edge_vec)^3;
    Jv(idx(4:6),idx(1:3)) = Jv(idx(4:6),idx(1:3)) - (eta*edge_len/2).*(edge_hat*u2' + u2*edge_hat'*eye(3))*(norm(edge_vec)^2*eye(3) - edge_vec*edge_vec')/norm(edge_vec)^3;
end
