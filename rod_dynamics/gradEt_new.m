%% Copyright M. Khalid Jawed (khalidjm@seas.ucla.edu)
% You should use this code at your own risk. Copy and redistribution is not
% permitted. Written permission is required.

%%
function [dF] = ...
    gradEt_new(n_dof, ind, node0, node1, node2, ...
    theta_e, theta_f, refTwist, ...
    bend_twist_spring, undef_refTwist)

l_k = bend_twist_spring.voronoiLen;
GJ = bend_twist_spring.stiff_GJ;

% Inputs:
% node0: 1x3 vector - position of the node prior to the "twisting" node
% node1: 1x3 vector - position of the "twisting" node
% node2: 1x3 vector - position of the node after the "twisting" node
%
% theta_e: scalar - twist angle of the first edge
% theta_f: scalar - twist angle of the second (last) edge
%
% l_k: voronoi length (undeformed) of the turning node
% GJ: scalar - twisting stiffness
%
% Outputs:
% dF: 11x1  vector - gradient of the twisting energy at node1.

%% Computation of gradient of the twist
% gradTwist = zeros(11,1);
gradTwist = zeros(n_dof,1);

ee = node1 - node0;
ef = node2 - node1;

norm_e = norm(ee);
norm_f = norm(ef);

norm2_e = norm_e^2;
norm2_f = norm_f^2;

te = ee / norm_e;
tf = ef / norm_f;

% Curvature binormal
kb = 2.0 * cross(te, tf) / (1.0 + dot(te, tf));
gradTwist(ind(1:3)) = -0.5 / norm_e * kb';
gradTwist(ind(7:9)) = 0.5 / norm_f * kb';
gradTwist(ind(4:6)) = -(gradTwist(ind(1:3))+gradTwist(ind(7:9)));
gradTwist(ind(10)) = -1;
gradTwist(ind(11)) = 1;

%% Gradient of Et
integratedTwist = theta_f - theta_e + refTwist - undef_refTwist;
dF = (GJ/l_k * integratedTwist) .* gradTwist;

end
