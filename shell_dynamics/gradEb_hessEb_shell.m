function [dF, dJ] = gradEb_hessEb_shell(n_dof, ind, x0,x1,x2,x3, hinge_spring)
kb = hinge_spring.kb;
thetaBar = hinge_spring.thetaBar;


% //         x2
% //         /\
% //        /  \
% //     e1/    \e3
% //      /  t0  \
% //     /        \
% //    /    e0    \
% //  x0------------x1
% //    \          /
% //     \   t1   /
% //      \      /
% //     e2\    /e4
% //        \  /
% //         \/
% //         x3
% //
% // Edge orientation: e0,e1,e2 point away from x0
% //                      e3,e4 point away from x1

dF = zeros(n_dof,1);
dJ = zeros(n_dof,n_dof);

% Formulas: 
% E = 0.5 * kb * (theta-thetaBar)^2
% F = 0.5 * kb * (2 (theta-thetaBar) d theta/dx)
% J = dF/dx = 0.5 * kb * [ 2 (d theta / dx) transpose(d theta/dx) + 
%       2 (theta-thetaBar) (d^2 theta/ dx^2 ) ]

theta = getTheta(x0, x1, x2, x3);
grad = gradTheta(x0, x1, x2, x3);
dF_unit = 0.5 * kb * (2 * (theta-thetaBar) * grad);
dF(ind) = dF_unit;

hess = hessTheta(x0, x1, x2, x3);
dJ_unit = 0.5 * kb * ( 2 * grad * transpose(grad) + 2 * (theta-thetaBar) * hess );
dJ(ind,ind) = dJ_unit;

end