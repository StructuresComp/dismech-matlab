function [ts, fs, cs] = calculate_ts_fs_cs(pi, pj, pk, tau_i0, tau_j0, tau_k0)
% edges
vi = pk - pj ; % 3*1 edge i vector
vj = pi - pk ; % 3*1 edge j vector
vk = pj - pi ; % 3*1 edge k vector

% edge lengths
li = norm(vi);
lj = norm(vj);
lk = norm(vk);

% triangle face normal
normal = cross(vk, vi);
A = norm(normal)/2; % area of triangular face
unit_norm = normal/norm(normal); % normalized triangle face normal vector

% t_i's (tangent (perpendicular to edge, in plane of triangle) of length =
% |vi|)
t_i = cross(vi,unit_norm);
t_j = cross(vj,unit_norm);
t_k = cross(vk,unit_norm);

% c_i's :  scalars
c_i = 1/( A*li*dot((t_i/norm(t_i)),tau_i0) );
c_j = 1/( A*lj*dot((t_j/norm(t_j)),tau_j0) );
c_k = 1/( A*lk*dot((t_k/norm(t_k)),tau_k0) );

% f_i's :  scalars
f_i = dot(unit_norm,tau_i0);
f_j = dot(unit_norm,tau_j0);
f_k = dot(unit_norm,tau_k0);

fs = [f_i, f_j, f_k]; % (1*3)

ts = [t_i , t_j , t_k]; % t_i are columns

cs = [c_i, c_j, c_k]; % (1*3)

end
