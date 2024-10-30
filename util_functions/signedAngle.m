function angle = signedAngle( u, v, n )
% "angle" is signed angle from vector "u" to vector "v" with axis "n"
w = cross(u,v);
angle = atan2( norm(w), dot(u,v) );
if (dot(n,w) < 0) 
    angle = -angle;
end

% if angle is even multiple of pi, its basically zero? but what to do in
% case of continuous unidirectional twisting
% if (mod(angle, 2*pi)<1e-3)
%     angle
%     % angle =0;
% end
end

% C++ code
% inline Scalar signedAngle(const Vec3d& u, const Vec3d& v, const Vec3d& n)
% {
%   Vec3d w = u.cross(v);
%   Scalar angle = atan2(w.norm(), u.dot(v));
%   if (n.dot(w) < 0) return -angle;
%   return angle;
% }