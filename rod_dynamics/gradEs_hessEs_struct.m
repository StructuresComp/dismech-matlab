function [dF, dJ] = ...
    gradEs_hessEs_struct(n_dof, ind, node0p, node1p, stretch_spring)

l_k = stretch_spring.refLen;
EA = stretch_spring.stiff;

% Inputs:
% node0: 1x3 vector - position of the first node
% node1: 1x3 vector - position of the last node
%
% l_k: reference length (undeformed) of the edge
% EA: scalar - stretching stiffness - Young's modulus times area
%
% Outputs:
% dF: n_dofx1  vector - gradient of the stretching energy between node0 and node1.
% dJ: n_dofxn_dof vector - hessian of the stretching energy between node0 and node1.

dF = zeros(n_dof,1);
dJ = zeros(n_dof,n_dof);

%% Gradient of Es
edge = (node1p - node0p)'; % 3x1 edge vector
edgeLen = norm(edge);
tangent = edge / edgeLen;
epsX = edgeLen/l_k - 1;
dF_unit = EA * tangent * epsX;

dF(ind(1:3)) = - dF_unit;
dF(ind(4:6)) =   dF_unit;

%% Hessian of Es
Id3 = eye(3);
M = EA * ( ...
    (1/l_k - 1/edgeLen) * Id3 + ...
    1/edgeLen * (edge*edge')/ edgeLen^2 ...
    ); % Note edge * edge' must be 3x3

dJ(ind(1:3), ind(1:3)) = M;
dJ(ind(4:6), ind(4:6)) = M;
dJ(ind(1:3), ind(4:6)) = - M;
dJ(ind(4:6), ind(1:3)) = - M;

end
