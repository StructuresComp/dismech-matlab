function theta = getTheta(x0, x1, x2, x3)

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

n0 = cross(m_e0, m_e1);
n1 = cross(m_e2, m_e0);

% theta = atan2(norm(cross(n0,n1)), dot(n0,n1));
theta = signedAngle(n0, n1, m_e0);

end

function angle = signedAngle( u, v, n )
w = cross(u,v);
angle = atan2( norm(w), dot(u,v) );
if (dot(n,w) < 0) 
    angle = -angle;
end
end