function Fs = getFs(MultiRod, stretch_springs, q)

n_stretch = numel(stretch_springs);
n_DOF = MultiRod.n_DOF;

Fs = zeros(n_DOF,1);

for c = 1:n_stretch
    n0=stretch_springs(c).nodes_ind(1);
    n1=stretch_springs(c).nodes_ind(2);
    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    ind = stretch_springs(c).ind;
    dF = gradEs(n_DOF, ind, node0p, node1p, stretch_springs(c));
    Fs(ind) = Fs(ind) - dF(ind);
 
end
end
