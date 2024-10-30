function Fs = getFs(MultiRod, stretch_springs, q)

global bug 

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

    %% to debug
    for i=1:numel(dF)
        if (dF(i)~= 0 && ~find(ind==i))
            fprintf("Bug: dF getting changed at wrong indices")
            bug=1;
        end                                      
    end
 
end
end
