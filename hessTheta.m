function hessTheta = hessTheta(x0, x1, x2, x3)

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
    

%% Gradient of Theta
gradTheta = zeros(12, 1);
gradTheta(1:3) = m_cosA3 * m_nn1 / m_h3 + m_cosA4 * m_nn2 / m_h4;
gradTheta(4:6) = m_cosA1 * m_nn1 / m_h1 + m_cosA2 * m_nn2 / m_h2;
gradTheta(7:9) = - m_nn1 / m_h01;
gradTheta(10:12) = - m_nn2 / m_h02;

%%

m_m1 =  cross(m_nn1, m_e1) / norm(m_e1);
m_m2 = -cross(m_nn2, m_e2) / norm(m_e2);
m_m3 = -cross(m_nn1, m_e3) / norm(m_e3);
m_m4 =  cross(m_nn2, m_e4) / norm(m_e4);
m_m01 =-cross(m_nn1, m_e0) / norm(m_e0);
m_m02 = cross(m_nn2, m_e0) / norm(m_e0);
    

%% Hessian of Theta
M331  = m_cosA3 / (m_h3 * m_h3) * m_m3 * transpose(m_nn1);
M311  = m_cosA3 / (m_h3 * m_h1) * m_m1 * transpose(m_nn1);
M131  = m_cosA1 / (m_h1 * m_h3) * m_m3 * transpose(m_nn1);
M3011 = m_cosA3 / (m_h3 * m_h01) * m_m01 * transpose(m_nn1);
M111  = m_cosA1 / (m_h1 * m_h1) * m_m1 * transpose(m_nn1);
M1011 = m_cosA1 / (m_h1 * m_h01) * m_m01 * transpose(m_nn1);


M442 = m_cosA4 / (m_h4 * m_h4) * m_m4 * transpose(m_nn2);
M422 = m_cosA4 / (m_h4 * m_h2) * m_m2 * transpose(m_nn2);
M242 = m_cosA2 / (m_h2 * m_h4) * m_m4 * transpose(m_nn2);
M4022 = m_cosA4 / (m_h4 * m_h02) * m_m02 * transpose(m_nn2);
M222 = m_cosA2 / (m_h2 * m_h2) * m_m2 * transpose(m_nn2);
M2022 = m_cosA2 / (m_h2 * m_h02) * m_m02 * transpose(m_nn2);

B1 = 1 / power(norm(m_e0), 2) * m_nn1 * transpose(m_m01);
B2 = 1 / power(norm(m_e0), 2) * m_nn2 * transpose(m_m02);

N13 = 1 / (m_h01 * m_h3) * m_nn1 * transpose(m_m3);
N24 = 1 / (m_h02 * m_h4) * m_nn2 * transpose(m_m4);
N11 = 1 / (m_h01 * m_h1) * m_nn1 * transpose(m_m1);
N22 = 1 / (m_h02 * m_h2) * m_nn2 * transpose(m_m2);
N101 = 1 / (m_h01 * m_h01) * m_nn1 * transpose(m_m01);
N202 = 1 / (m_h02 * m_h02) * m_nn2 * transpose(m_m02);

hessTheta = zeros(12,12);

hessTheta(1:3, 1:3) = MMT(M331) - B1 + MMT(M442) - B2;
hessTheta(1:3, 4:6) = M311 + transpose(M131) + B1 + M422 + transpose(M242) + B2;
hessTheta(1:3, 7:9) = M3011 - N13;
hessTheta(1:3, 10:12) = M4022 - N24;
hessTheta(4:6, 4:6) = MMT(M111) - B1 + MMT(M222) - B2;
hessTheta(4:6, 7:9) = M1011 - N11;
hessTheta(4:6, 10:12) = M2022 - N22;
hessTheta(7:9, 7:9) = -MMT(N101);
hessTheta(10:12, 10:12) = -MMT(N202);

% symmetric matrix
hessTheta(4:6, 1:3) = transpose(hessTheta(1:3, 4:6));
hessTheta(7:9, 1:3) = transpose(hessTheta(1:3, 7:9));
hessTheta(10:12, 1:3) = transpose(hessTheta(1:3, 10:12));
hessTheta(7:9, 4:6) = transpose(hessTheta(4:6, 7:9));
hessTheta(10:12, 4:6) = transpose(hessTheta(4:6, 10:12));
    
end

function matMatTrans = MMT(mat)
matMatTrans = mat + transpose(mat);
end