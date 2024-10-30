function [Fv, Jv] = getViscousForce(q,q0,dt, eta, MultiRod)
u = (q-q0)/dt ;
Fv = zeros(MultiRod.n_DOF,1);
Jv = zeros(MultiRod.n_DOF,MultiRod.n_DOF);
for i = 1:MultiRod.n_nodes
    idx = mapNodetoDOF(i);
    Fv(idx)  = -(eta*MultiRod.voronoiRefLen(i)).*u(idx);
    Jv(idx,idx) = -(eta*MultiRod.voronoiRefLen(i)/dt).*eye(3);
end

