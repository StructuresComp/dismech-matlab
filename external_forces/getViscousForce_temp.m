function [Fv, Jv] = getViscousForce_temp(q,q0,dt, eta, MultiRod)
u = (q-q0)/dt ;
Fv = zeros(MultiRod.n_DOF,1);
Jv = zeros(MultiRod.n_DOF,MultiRod.n_DOF);
for i = 1:MultiRod.n_nodes
    idx = mapNodetoDOF(i);
    Fv(idx)  = -(eta*0.005).*u(idx);
    Jv(idx,idx) = -(eta*0.005/dt).*eye(3);
end

