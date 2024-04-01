function d = parallel_transport(u, t1, t2)
% This function parallel transports a vector u from tangent t1 to t2
% Input:
% t1 - vector denoting the first tangent
% t2 - vector denoting the second tangent
% u - vector that needs to be parallel transported
% Output:
% d - vector after parallel transport

b = cross(t1, t2);
if (norm(b) == 0 ) 
    d = u;
else
    b = b / norm(b);
    % The following four lines may seem unnecessary but can sometimes help
    % with numerical stability
    b = b - dot(b,t1) * t1;
    b = b / norm(b);
    b = b - dot(b,t2) * t2;
    b = b / norm(b);
    
    n1 = cross(t1, b);
    n2 = cross(t2, b);
    d = dot(u,t1) * t2 + dot(u, n1) * n2 + dot(u, b) * b;
end
end
