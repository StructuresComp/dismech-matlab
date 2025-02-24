function [E_with_stiff, gradE_with_stiff, gradE, gradE2] = ...
    Eb_gradEb_shell_midedge ...
    (stiff, nu, pi, pj, pk, xi_i, xi_j, xi_k, s_i, s_j, s_k, tau_i0, tau_j0, tau_k0, A, ls, ...
    init_ts, init_cs, init_fs, init_xi, ...
    optional_t_i, optional_t_j, optional_t_k, optional_c_i, optional_c_j, optional_c_k)

% *************************************************************************
% Inputs:
% pi, pj, pk: vertex DOF - 3*1 position vectors of vertices of one triangle
%             face in the mesh
% xi_i, xi_j, xi_k: edge DOF - scalar DOF corresponding to mid-edge normal -
%                   projection of the mid-edge normal of edge i on tau_i^0
% s_i, s_j, s_k: sign (+/-) corresponding to each of the xi_i DOFs
% tau_i0, tau_j0, tau_k0: 3*1 vectors
% A : intial value of triangle area
% init_ts: initial value of t vectors of the triangle
% init_cs: initial value of c for each edge of the triangle
% init_xi: initial value of xi for each edge of the triangle

% Outputs:
% E: Energy - scalar, elastic energy of the triangle face
% gradE: Gradient of Shell Energy - vector 12*1
% hessE: Hessian of Shell Energy - matrix 12*12

% Functions used:
% delfi_by_delpk.m
% ddel_fi_by_del_p_k1_p_k2.m
% *************************************************************************
% change signs of tau_0 according to edge vectors ownership
tau_i0 = s_i*tau_i0;
tau_j0 = s_j*tau_j0;
tau_k0 = s_k*tau_k0;

% edges
vi = pk - pj ; % 3*1 edge i vector
vj = pi - pk ; % 3*1 edge j vector
vk = pj - pi ; % 3*1 edge k vector

% triangle face normal
normal = cross(vk, vi);
% actual_A = norm(normal)/2; % area of triangular face
unit_norm = normal/norm(normal); % normalized triangle face normal vector

% t_i's (tangent (perpendicular to edge, in plane of triangle) of length =
% |vi|)
actual_t_i = cross(vi,unit_norm);
actual_t_j = cross(vj,unit_norm);
actual_t_k = cross(vk,unit_norm);

% actual_ts = [actual_t_i, actual_t_j, actual_t_k];

% c_i's :  scalars
actual_c_i = 1/( A*ls(1)*dot((actual_t_i/norm(actual_t_i)),tau_i0) );
actual_c_j = 1/( A*ls(2)*dot((actual_t_j/norm(actual_t_j)),tau_j0) );
actual_c_k = 1/( A*ls(3)*dot((actual_t_k/norm(actual_t_k)),tau_k0) );

% if optional inputs are given - use them, else calculate ti's and ci's
if nargin>20
    % fprintf('optional input arguments given')
    t_i = optional_t_i;
    t_j = optional_t_j;
    t_k = optional_t_k;
    c_i = optional_c_i;
    c_j = optional_c_j;
    c_k = optional_c_k;
else
    t_i = actual_t_i;
    t_j = actual_t_j;
    t_k = actual_t_k;
    c_i = actual_c_i;
    c_j = actual_c_j;
    c_k = actual_c_k;
end

% f_i's :  scalars
f_i = dot(unit_norm,tau_i0);
f_j = dot(unit_norm,tau_j0);
f_k = dot(unit_norm,tau_k0);

f = [f_i, f_j, f_k]; % (1*3)

t = [t_i , t_j , t_k]; % t_i are columns

c = [c_i, c_j, c_k]; % (1*3)

s = [s_i, s_j, s_k]; % scalars (1*3)
xi = [xi_i, xi_j, xi_k]; % scalars (1*3)
tau_0 = [tau_i0, tau_j0, tau_k0]; % (3*3) tau_is are column vectors 

%% Shell Energy: trace (shape operator^2) + trace^2 (shape operator)

E = 0; % initialize scalar
E2 = 0; % initialize scalar

for i=1:3
    for j=1:3
        E = E + ( (c(i)*c(j)) * (s(i)*xi(i) - f(i)) * (s(j)*xi(j) - f(j)) * (dot(t(:,i),t(:,j))^2) ) + ...
            ( (init_cs(i)*init_cs(j)) * (s(i)*init_xi(i) - init_fs(i)) * (s(j)*init_xi(j) - init_fs(j)) * (dot(init_ts(:,i),init_ts(:,j))^2) ) - ...
            2 * ( (c(i)*init_cs(j)) * (s(i)*xi(i) - f(i)) * (s(j)*init_xi(j) - init_fs(j)) * (dot(t(:,i),init_ts(:,j))^2) );

        E2 = E2 + ( (c(i)*c(j)) * ((norm(t(:,i))^2)*(norm(t(:,j))^2)) * (s(i)*xi(i) - f(i)) * (s(j)*xi(j) - f(j)) ) + ...
            ( (init_cs(i)*init_cs(j)) * ((ls(i)^2)*(ls(j)^2)) * (s(i)*init_xi(i) - init_fs(i)) * (s(j)*init_xi(j) - init_fs(j)) ) - ...
            2 * ( (c(i)*init_cs(j)) * ((norm(t(:,i))^2)*((ls(j))^2)) * (s(i)*xi(i) - f(i)) * (s(j)*init_xi(j) - init_fs(j)) );
    end
end
E_with_stiff = stiff*(nu*E + (1-nu)*E2).*A ;

%% Gradient of Energy

% initialize
del_E_del_pi = zeros(1,3);
del_E_del_pj = zeros(1,3);
del_E_del_pk = zeros(1,3);

del_E_del_xi_i = 0;
del_E_del_xi_j = 0;
del_E_del_xi_k = 0;

%
del_E2_del_pi = zeros(1,3);
del_E2_del_pj = zeros(1,3);
del_E2_del_pk = zeros(1,3);

del_E2_del_xi_i = 0;
del_E2_del_xi_j = 0;
del_E2_del_xi_k = 0;

% compute
for i=1:3
    for j=1:3
        del_E_del_pi = del_E_del_pi - c(i)*c(j) * ((s(i)*xi(i) - f(i)) .* delfi_by_delpk(tau_0(:,j), t_i, unit_norm, A) + ...
            (s(j)*xi(j) - f(j)) .* delfi_by_delpk(tau_0(:,i), t_i, unit_norm, A)) .* (dot(t(:,i),t(:,j))^2) - ...
            2* c(i) * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * delfi_by_delpk(tau_0(:,i), t_i, unit_norm, A) * (dot(t(:,i),init_ts(:,j)))^2 ;

        del_E_del_pj = del_E_del_pj - c(i)*c(j) * ((s(i)*xi(i) - f(i)) .* delfi_by_delpk(tau_0(:,j), t_j, unit_norm, A) + ...
            (s(j)*xi(j) - f(j)) .* delfi_by_delpk(tau_0(:,i), t_j, unit_norm, A)) .* (dot(t(:,i),t(:,j))^2) - ...
            2* c(i) * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * delfi_by_delpk(tau_0(:,i), t_j, unit_norm, A) * (dot(t(:,i),init_ts(:,j)))^2 ;

        del_E_del_pk = del_E_del_pk - c(i)*c(j) * ((s(i)*xi(i) - f(i)) .* delfi_by_delpk(tau_0(:,j), t_k, unit_norm, A) + ...
            (s(j)*xi(j) - f(j)) .* delfi_by_delpk(tau_0(:,i), t_k, unit_norm, A)) .* (dot(t(:,i),t(:,j))^2) - ...
            2* c(i) * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * delfi_by_delpk(tau_0(:,i), t_k, unit_norm, A) * (dot(t(:,i),init_ts(:,j)))^2 ;


        del_E2_del_pi = del_E2_del_pi - c(i)*c(j) * ((s(i)*xi(i) - f(i)) .* delfi_by_delpk(tau_0(:,j), t_i, unit_norm, A) + ...
            (s(j)*xi(j) - f(j)) .* delfi_by_delpk(tau_0(:,i), t_i, unit_norm, A)) .* (norm(t(:,i))^2*norm(t(:,j))^2) + ...
            2* c(i) * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * delfi_by_delpk(tau_0(:,i), t_i, unit_norm, A) .* (norm(t(:,i))^2*ls(j)^2);

        del_E2_del_pj = del_E2_del_pj - c(i)*c(j) * ((s(i)*xi(i) - f(i)) .* delfi_by_delpk(tau_0(:,j), t_j, unit_norm, A) + ...
            (s(j)*xi(j) - f(j)) .* delfi_by_delpk(tau_0(:,i), t_j, unit_norm, A)) .* (norm(t(:,i))^2*norm(t(:,j))^2) + ...
            2* c(i) * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * delfi_by_delpk(tau_0(:,i), t_j, unit_norm, A) .* (norm(t(:,i))^2*ls(j)^2);


        del_E2_del_pk = del_E2_del_pk - c(i)*c(j) * ((s(i)*xi(i) - f(i)) .* delfi_by_delpk(tau_0(:,j), t_k, unit_norm, A) + ...
            (s(j)*xi(j) - f(j)) .* delfi_by_delpk(tau_0(:,i), t_k, unit_norm, A)) .* (norm(t(:,i))^2*norm(t(:,j))^2) + ...
            2* c(i) * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * delfi_by_delpk(tau_0(:,i), t_k, unit_norm, A) .* (norm(t(:,i))^2*ls(j)^2);

    end 
end

for j=1:3
        del_E_del_xi_i = del_E_del_xi_i + (2*c_i*s_i) * c(j) * (s(j)*xi(j) - f(j)) * (dot(t_i, t(:,j))^2) ...
            + 2*c_i*s_i * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * (dot(t_i,init_ts(:,j)))^2 ;
        del_E_del_xi_j = del_E_del_xi_j + (2*c_j*s_j) * c(j) * (s(j)*xi(j) - f(j)) * (dot(t_j, t(:,j))^2) ...
            + 2*c_i*s_i * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * (dot(t_i,init_ts(:,j)))^2 ;
        del_E_del_xi_k = del_E_del_xi_k + (2*c_k*s_k) * c(j) * (s(j)*xi(j) - f(j)) * (dot(t_k, t(:,j))^2) ...
            + 2*c_i*s_i * init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * (dot(t_i,init_ts(:,j)))^2 ;


        del_E2_del_xi_i = del_E2_del_xi_i + (2*c_i*s_i* norm(t_i)^2) * (c(j) * (s(j)*xi(j) - f(j)) * norm(t(:,j))^2 - ...
             init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * ls(j)^2);

        del_E2_del_xi_j = del_E2_del_xi_j + (2*c_j*s_j* norm(t_j)^2) * (c(j) * (s(j)*xi(j) - f(j)) * norm(t(:,j))^2 - ...
             init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * ls(j)^2);

        del_E2_del_xi_k = del_E2_del_xi_k + (2*c_k*s_k* norm(t_k)^2) * (c(j) * (s(j)*xi(j) - f(j)) * norm(t(:,j))^2 - ...
             init_cs(j) * (s(j)*init_xi(j) - init_fs(j)) * ls(j)^2);        
end

% collect in the gradient vector in the sequence: del_by [pi, pj, pk, xi_i, xi_j, xi_k]
% size = (12*1)
gradE = [del_E_del_pi , del_E_del_pj , del_E_del_pk , ...
    del_E_del_xi_i , del_E_del_xi_j , del_E_del_xi_k];

gradE2 = [del_E2_del_pi , del_E2_del_pj , del_E2_del_pk , ...
    del_E2_del_xi_i , del_E2_del_xi_j , del_E2_del_xi_k];

gradE_with_stiff = stiff.*(nu.*gradE + (1-nu).*gradE2).*A;
end