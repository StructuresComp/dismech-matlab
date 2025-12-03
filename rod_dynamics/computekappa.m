function kappa_L = computekappa(node0, node1, node2, m1e, m2e, m1f, m2f)
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
% Outputs:
% kappa: 1x2 vector - curvature at the turning node

t0 = (node1-node0) / norm(node1-node0);
t1 = (node2-node1) / norm(node2-node1);
% %% very imp check
% % if t0 and t1 are pointed exactly opposite,
% if abs(1.0 + dot(t0, t1)) <= 1e-6
%      t0
% end


kb = 2.0 * cross(t0, t1) / (1.0 + dot(t0, t1));

kappa_L = zeros(1, 2);
kappa1 = 0.5 * dot( kb, m2e + m2f);
kappa2 = -0.5 * dot( kb, m1e + m1f);
kappa_L(1) = kappa1;
kappa_L(2) = kappa2;

% if any(isnan(kappa_L))
%     kappa_L
% end

end