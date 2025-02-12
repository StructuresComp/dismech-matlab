function angle = signedAngle( u, v, n )
% "angle" is signed angle from vector "u" to vector "v" with axis "n"
w = cross(u,v);
angle = atan2( norm(w), dot(u,v) );
if (dot(n,w) < 0) 
    angle = -angle;
end
end