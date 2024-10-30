function Fpt = addPointForce(Fpt_ext, node_ind, MultiRod)
Fpt = zeros(MultiRod.n_DOF,1);
Fpt(mapNodetoDOF(node_ind)) = Fpt_ext;
end