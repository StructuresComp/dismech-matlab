function [Fb_shell, Jb_shell, hinge_springs] = getFb_shell(MultiRod, hinge_springs, q)

% global bug 

n_hinge = numel(hinge_springs);
n_DOF = MultiRod.n_DOF;

Fb_shell = zeros(n_DOF,1);
Jb_shell = zeros(n_DOF);

for c = 1:n_hinge
    n0=hinge_springs(c).nodes_ind(1);
    n1=hinge_springs(c).nodes_ind(2);
    n2=hinge_springs(c).nodes_ind(3);
    n3=hinge_springs(c).nodes_ind(4);

    x0 = q(mapNodetoDOF(n0));
    x1 = q(mapNodetoDOF(n1));
    x2 = q(mapNodetoDOF(n2));
    x3 = q(mapNodetoDOF(n3));
    
    ind = hinge_springs(c).ind;
    [dF, dJ] = ...
    gradEb_hessEb_shell(n_DOF, ind, x0,x1,x2,x3, hinge_springs(c));

    Fb_shell(ind) = Fb_shell(ind) - dF(ind);
    Jb_shell(ind, ind) = Jb_shell(ind, ind) - dJ(ind, ind);

    %% to debug
%     for i=1:numel(dF)
%         if (dF(i)~= 0 && ~find(ind==i))
%             fprintf("Bug: dF getting changed at wrong indices")
%             bug=1;
%         end
%     for j=1:numel(dF)
%         if (dJ(i,j)~= 0 && (~find(ind==i) || ~find(ind==j)))
%             fprintf("Bug: dJ getting changed at wrong indices")
%             bug=1;
%         end
%     end

%     end
    %% update spring forces in the spring structs
    hinge_springs(c).dF = dF(ind);
    hinge_springs(c).dJ = dJ(ind, ind);
 
end
