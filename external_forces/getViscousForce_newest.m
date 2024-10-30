function [Fv, Jv] = getViscousForce_newest(q,q0,dt,eta, MultiRod)
u = (q-q0)/dt ;
Fv = zeros(MultiRod.n_DOF,1);
Jv = zeros(MultiRod.n_DOF,MultiRod.n_DOF);

for c=1:MultiRod.n_edges 
    node1ind = MultiRod.Edges(c,1);
    node2ind = MultiRod.Edges(c,2);
    idx = [mapNodetoDOF(node1ind); mapNodetoDOF(node2ind)];
    edge_vec = q(mapNodetoDOF(node2ind))-q(mapNodetoDOF(node1ind));
    edge_len =  norm(edge_vec);
    edge_hat = edge_vec/edge_len;

    Fv(idx(1:3)) = Fv(idx(1:3)) + (eta*edge_len/2).* ...
        -( u(mapNodetoDOF(node1ind)) - dot(u(mapNodetoDOF(node1ind)), edge_hat).*edge_hat );
    Fv(idx(4:6)) = Fv(idx(4:6)) + (eta*edge_len/2).* ...
        -( u(mapNodetoDOF(node2ind)) - dot(u(mapNodetoDOF(node2ind)), edge_hat).*edge_hat );

    Jv(idx(1:3),idx(1:3)) = Jv(idx(1:3),idx(1:3)) - (eta*edge_len/(2*dt)).*eye(3);
    Jv(idx(4:6),idx(4:6)) = Jv(idx(4:6),idx(4:6)) - (eta*edge_len/(2*dt)).*eye(3);
end
