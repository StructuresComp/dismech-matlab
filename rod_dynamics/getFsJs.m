function [Fs, Js, stretch_springs] = getFsJs(MultiRod, stretch_springs, q)

global bug 

n_stretch = numel(stretch_springs);
n_DOF = MultiRod.n_DOF;

Fs = zeros(n_DOF,1);
Js = zeros(n_DOF);

for c = 1:n_stretch
    n0=stretch_springs(c).nodes_ind(1);
    n1=stretch_springs(c).nodes_ind(2);
    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    ind = stretch_springs(c).ind;
    [dF, dJ] = ...
    gradEs_hessEs_struct(n_DOF, ind, node0p, node1p, stretch_springs(c));

    Fs(ind) = Fs(ind) - dF(ind);
    Js(ind, ind) = Js(ind, ind) - dJ(ind, ind);

    if(ismembc(c,[4,5,6,7]))
        fprintf("Fs for edge: %d" ,c)
        dF(ind)
    end

    %% to debug
    for i=1:numel(dF)
        if (dF(i)~= 0 && ~find(ind==i))
            fprintf("Bug: dF getting changed at wrong indices")
            bug=1;
        end
    for j=1:numel(dF)
        if (dJ(i,j)~= 0 && (~find(ind==i) || ~find(ind==j)))
            fprintf("Bug: dJ getting changed at wrong indices")
            bug=1;
        end
    end

    end
    %% update spring forces in the spring structs
    stretch_springs(c).dF = dF(ind);
    stretch_springs(c).dJ = dJ(ind, ind);
 
end
%% to debug parachute
if(sum(abs(Fs(4:6)-Fs(10:12)))>10^-6)
    (Fs(4:6)-Fs(10:12))
%     error("not same aerodynamic force on left and right side")
end
end
