%% Copyright M. Khalid Jawed (khalidjm@seas.ucla.edu)
% You should use this code at your own risk. Copy and redistribution is not
% permitted. Written permission is required.

%%
function [dF] = ...
    gradEb (n_dof, ind, node0, node1, node2, ...
    m1e, m2e, m1f, m2f, ...
    bend_twist_spring)

kappaBar = bend_twist_spring.kappaBar;
l_k = bend_twist_spring.voronoiLen;
EI1 = bend_twist_spring.stiff_EI(1);
EI2 = bend_twist_spring.stiff_EI(2);
%
% Inputs:
% node0: 1x3 vector - position of the node prior to the "turning" node
% node1: 1x3 vector - position of the "turning" node
% node2: 1x3 vector - position of the node after the "turning" node
%
% m1e: 1x3 vector - material director 1 of the edge prior to turning
% m2e: 1x3 vector - material director 2 of the edge prior to turning
% m1f: 1x3 vector - material director 1 of the edge after turning
% m2f: 1x3 vector - material director 2 of the edge after turning
%
% kappaBar: 1x2 vector - natural curvature at the turning node
% l_k: voronoi length (undeformed) of the turning node
% EI: scalar - bending stiffness
%
% Outputs:
% dF: 11x1  vector - gradient of the bending energy at node1.

%% Computation of gradient of the two curvatures
% gradKappa = zeros(11,2);
gradKappa = zeros(n_dof,2);

ee = node1 - node0;
ef = node2 - node1;

norm_e = norm(ee);
norm_f = norm(ef);

te = ee / norm_e; % 1x3 edge tangent vector
tf = ef / norm_f; % 1x3 edge tangent vector

% Curvature binormal
kb = 2.0 * cross(te, tf) / (1.0 + dot(te, tf)); % 1x3 vector

chi = 1.0 + dot(te, tf); % scalar
tilde_t = (te + tf) / chi; % 1x3 vector
tilde_d1 = (m1e + m1f) / chi; % 1x3 vector
tilde_d2 = (m2e + m2f) / chi; % 1x3 vector

% Curvatures
kappa1 = 0.5 * dot( kb, m2e + m2f); % scalar
kappa2 = -0.5 * dot( kb, m1e + m1f); % scalar

% kappa1 = kappa(c, 1);
% kappa2 = kappa(c, 2);

Dkappa1De = 1.0 / norm_e * (-kappa1 * tilde_t + cross(tf,tilde_d2));
Dkappa1Df = 1.0 / norm_f * (-kappa1 * tilde_t - cross(te,tilde_d2));

Dkappa2De = 1.0 / norm_e * (-kappa2 * tilde_t - cross(tf,tilde_d1));
Dkappa2Df = 1.0 / norm_f * (-kappa2 * tilde_t + cross(te,tilde_d1));

%%
gradKappa(ind(1:3), 1) = -Dkappa1De';
gradKappa(ind(4:6), 1) = (Dkappa1De - Dkappa1Df)';
gradKappa(ind(7:9), 1) = Dkappa1Df';

gradKappa(ind(1:3), 2) = -Dkappa2De';
gradKappa(ind(4:6), 2) = (Dkappa2De - Dkappa2Df)';
gradKappa(ind(7:9), 2) = Dkappa2Df';

gradKappa(ind(10), 1) = -0.5 * dot(kb, m1e);
gradKappa(ind(11), 1) = -0.5 * dot(kb, m1f);
gradKappa(ind(10), 2) = -0.5 * dot(kb, m2e);
gradKappa(ind(11), 2) = -0.5 * dot(kb, m2f);

%% Gradient of Eb
EIMat = [ EI1 0; ...
    0 EI2];
kappaVector = [kappa1 kappa2];
dkappaVector = kappaVector - kappaBar;
dF = gradKappa * EIMat * dkappaVector' / l_k;
end
