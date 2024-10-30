function [dF] = ...
    gradEs(n_dof, ind, node0p, node1p, stretch_spring)

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

dF = zeros(n_dof,1);

%% Gradient of Es
edge = (node1p - node0p)'; % 3x1 edge vector
edgeLen = norm(edge);
tangent = edge / edgeLen;
epsX = edgeLen/l_k - 1;
dF_unit = EA * tangent * epsX;

dF(ind(1:3)) = - dF_unit;
dF(ind(4:6)) =   dF_unit;

end
