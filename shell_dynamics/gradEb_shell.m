function [dF, dJ] = gradEb_shell(n_dof, ind, x0,x1,x2,x3, hinge_spring)
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


% % In the original code, there are probaly TWO sign errors in the expressions for m_h3 and m_h4.
% [Original code: % https://github.com/shift09/plates-shells/blob/master/src/bending.cpp]
% I indicated those two corrections by writing the word "CORRECTION" next
% to them.

dF = zeros(n_dof,1);
theta = getTheta(x0, x1, x2, x3);
grad = gradTheta(x0, x1, x2, x3);
dF_unit = 0.5 * kb * (2 * (theta-thetaBar) * grad);
dF(ind) = dF_unit;

end