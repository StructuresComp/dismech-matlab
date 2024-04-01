function grad = gradTheta(x0, x1, x2, x3)

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

if numel(x0) == 12 % Let us allow another type of inputs. In this case, x0 contains all the info.
    x1 = x0(4:6);
    x2 = x0(7:9);
    x3 = x0(10:12);
    x0 = x0(1:3);
end

m_e0 = x1 - x0;
m_e1 = x2 - x0;
m_e2 = x3 - x0;
m_e3 = x2 - x1;
m_e4 = x3 - x1;

m_cosA1 =   dot( m_e0, m_e1 ) / ( norm(m_e0) * norm(m_e1));
m_cosA2 =   dot( m_e0, m_e2 ) / ( norm(m_e0) * norm(m_e2));
m_cosA3 = - dot( m_e0, m_e3 ) / ( norm(m_e0) * norm(m_e3));
m_cosA4 = - dot( m_e0, m_e4 ) / ( norm(m_e0) * norm(m_e4));

m_sinA1 =   norm( cross(m_e0, m_e1) ) / ( norm(m_e0) * norm(m_e1));
m_sinA2 =   norm( cross(m_e0, m_e2) ) / ( norm(m_e0) * norm(m_e2));
m_sinA3 =  -norm( cross(m_e0, m_e3) ) / ( norm(m_e0) * norm(m_e3));
m_sinA4 =  -norm( cross(m_e0, m_e4) ) / ( norm(m_e0) * norm(m_e4));

m_nn1 =  cross( m_e0, m_e3);
m_nn1 =  m_nn1 / norm(m_nn1);
m_nn2 = -cross( m_e0, m_e4);
m_nn2 =  m_nn2 / norm(m_nn2);

m_h1 =  norm(m_e0) * m_sinA1;
m_h2 =  norm(m_e0) * m_sinA2;
m_h3 =  - norm(m_e0) * m_sinA3; % CORRECTION
m_h4 =  - norm(m_e0) * m_sinA4; % CORRECTION
m_h01 = norm(m_e1) * m_sinA1;
m_h02 = norm(m_e2) * m_sinA2;
    

%%
gradTheta = zeros(12, 1);

gradTheta(1:3) = m_cosA3 * m_nn1 / m_h3 + m_cosA4 * m_nn2 / m_h4;
gradTheta(4:6) = m_cosA1 * m_nn1 / m_h1 + m_cosA2 * m_nn2 / m_h2;
gradTheta(7:9) = - m_nn1 / m_h01;
gradTheta(10:12) = - m_nn2 / m_h02;

%%
grad = gradTheta;

end