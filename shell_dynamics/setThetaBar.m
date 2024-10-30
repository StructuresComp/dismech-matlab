function hinge_springs = setThetaBar(hinge_springs, MultiRod)
n_hinge = size(hinge_springs,1);
for c=1:n_hinge
    n0=hinge_springs(c).nodes_ind(1);
    n1=hinge_springs(c).nodes_ind(2);
    n2=hinge_springs(c).nodes_ind(3);
    n3=hinge_springs(c).nodes_ind(4);

    x0 = MultiRod.q(mapNodetoDOF(n0));
    x1 = MultiRod.q(mapNodetoDOF(n1));
    x2 = MultiRod.q(mapNodetoDOF(n2));
    x3 = MultiRod.q(mapNodetoDOF(n3));

    theta_bar = getTheta(x0,x1,x2,x3);

%     if(theta_bar==pi) 
%         theta_bar = 0; % check why this is needed
%     end

    hinge_springs(c).thetaBar = theta_bar;
end
end