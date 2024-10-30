function vNew = rotateAxisAngle( v, z, theta )
    if (theta == 0)
     vNew = v;
    else
     vNew = cos(theta)*v + sin(theta)*cross(z,v) + dot(z,v)*(1-cos(theta))*z;
    end
end