% check analytical gradient against fdm

clear all;
close all;
clc;

addpath ../util_functions/
addpath ../shell_dynamics/
fprintf('FDM verification of midedge normal based shell bending\n');

stiff = 1;
nu = 0.5;

%% element: triangle

% initial values (non-zero init curvature)

init_ps = rand(3,3);
init_p1 = init_ps(:,1);
init_p2 = init_ps(:,2);
init_p3 = init_ps(:,3);

init_p1_ = rand(3,1);
init_p2_ = rand(3,1);
init_p3_ = rand(3,1);

% init_xi_1 = rand;
% init_xi_2 = rand;
% init_xi_3 = rand;

% % initial values (zero init curvature)

% init_ps = [rand(2,3);0,0,0];
% init_p1 = init_ps(:,1);
% init_p2 = init_ps(:,2);
% init_p3 = init_ps(:,3);
% 
% init_p1_ = [rand(2,1);0];
% init_p2_ = [rand(2,1);0];
% init_p3_ = [rand(2,1);0];

nodes = [init_p1, init_p2, init_p3, init_p1_, init_p2_, init_p3_]';
edges = [2,3; 3,1; 1,2; 2,4; 3,5; 1,6;...
    3,4; 1,5; 2,6];

figure()
hold on;
for i=1:size(edges,1)
    n1 = edges(i,1);
    n2 = edges(i,2);
    n1pos = nodes(n1,:);
    n2pos = nodes(n2,:);
   
    plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ko-');
end
hold off;

%%

init_v1 = init_p3 - init_p2;
init_v2 = init_p1 - init_p3; 
init_v3 = init_p2 - init_p1;

% normal of the triangle
normal_init = cross((init_p2-init_p1), (init_p3-init_p2));
A_init = norm(normal_init)/2;
n_init = normal_init/norm(normal_init);

% to calculate normals of adjacent triangles:

% edges of the adjacent triangles
e1 = init_p1_-init_p2;
e2 = init_p2_-init_p3;
e3 = init_p3_-init_p1;

% normals of adjacent triangles
n_2_init = cross(e1,init_v1);
n_3_init = cross(e2,init_v2);
n_4_init = cross(e3,init_v3);

n_2_init = n_2_init/norm(n_2_init);
n_3_init = n_3_init/norm(n_3_init);
n_4_init = n_4_init/norm(n_4_init);

% n1_avg associated with each of the edges
n1_avg_init = (n_init+n_2_init)/2;
if norm(n1_avg_init)<=1e-4
    n1_avg_init = (n_init-n_2_init)/2;
end
n2_avg_init = (n_init+n_3_init)/2;
if norm(n2_avg_init)<=1e-4
    n2_avg_init = (n_init-n_3_init)/2;
end
n3_avg_init = (n_init+n_4_init)/2;
if norm(n3_avg_init)<=1e-4
    n3_avg_init = (n_init-n_4_init)/2;
end

n1_avg_init = n1_avg_init/ norm(n1_avg_init);
n2_avg_init = n2_avg_init/ norm(n2_avg_init);
n3_avg_init = n3_avg_init/ norm(n3_avg_init);

% tau_is
tau1_0 = cross(init_v1, n1_avg_init);
tau2_0 = cross(init_v2, n2_avg_init);
tau3_0 = cross(init_v3, n3_avg_init);

% sign coefficients
s_1 = 1;
s_2 = 1;
s_3 = 1;

init_xi_1 = dot(n1_avg_init, tau1_0);
init_xi_2 = dot(n2_avg_init, tau2_0);
init_xi_3 = dot(n3_avg_init, tau3_0);

init_xis = [init_xi_1, init_xi_2, init_xi_3]; 

[init_ts, init_fs, init_cs] = calculate_ts_fs_cs(init_p1, init_p2, init_p3, tau1_0, tau2_0, tau3_0);

%% deformed
% vertex point vectors: p_i,j,k
ps = init_ps + 0.001*rand(3,3);
p1 = ps(:,1) ;
p2 = ps(:,2) ;
p3 = ps(:,3) ;

xis = init_xis + 0.001*rand(1,3);
xi_1 = xis(1);
xi_2 = xis(2);
xi_3 = xis(3);

p1_ = p1 + 0.001*rand(3,1);
p2_ = p2 + 0.001*rand(3,1);
p3_ = p3 + 0.001*rand(3,1);

% edge vectors: v_i,j,k
v1 = p3 - p2 ; 
v2 = p1 - p3 ;
v3 = p2 - p1 ;

% normal of the triangle
normal = cross((p2-p1), (p3-p2));
A = norm(normal)/2;
n = normal/norm(normal);

% to calculate normals of adjacent triangles:

% edges of the adjacent triangles
e1 = p1_-p2;
e2 = p2_-p3;
e3 = p3_-p1;

n_2 = cross(e1,v1);
n_3 = cross(e2,v2);
n_4 = cross(e3,v3);

n_2 = n_2/norm(n_2);
n_3 = n_3/norm(n_3);
n_4 = n_4/norm(n_4);

% n1_avg associated with each of the edges
n1_avg = (n+n_2)/2;
if norm(n1_avg)<=1e-4
    n1_avg = (n-n_2)/2;
end
n2_avg = (n+n_3)/2;
if norm(n2_avg)<=1e-4
    n2_avg = (n-n_3)/2;
end
n3_avg = (n+n_4)/2;
if norm(n3_avg)<=1e-4
    n3_avg = (n-n_4)/2;
end

n1_avg = n1_avg/ norm(n1_avg);
n2_avg = n2_avg/ norm(n2_avg);
n3_avg = n3_avg/ norm(n3_avg);

tau1_0 = cross(v1, n1_avg);
tau2_0 = cross(v2, n2_avg);
tau3_0 = cross(v3, n3_avg);

% [E, gradE, hessE,~,~,~,~,~,~, t1, t2, t3, c1, c2, c3] = ...
%     Energy_Grad_Hess_with2terms_nat_curv ...
%     (stiff, nu, p1, p2, p3, xi_1, xi_2, xi_3, s_1, s_2, s_3, tau1_0, tau2_0, tau3_0, ...
%     init_ts, init_cs, init_fs, init_xis);

%%
% [E_with_stiff, gradE_with_stiff, hessE_with_stiff, gradE_FDM, hessE_FDM] = ...
%     Debug_Eb_gradEb_FDM_hessEb_shell_midedge ...
%     (stiff, nu, p1, p2, p3, xi_1, xi_2, xi_3, s_1, s_2, s_3, tau1_0, tau2_0, tau3_0, A, ls, ...
%     init_ts, init_cs, init_fs, init_xis)

ls = [norm(v1); norm(v2); norm(v2)];
    [E, gradE, hessE, t_i, t_j, t_k, c_i, c_j, c_k] = Eb_gradEb_hessEb_shell_midedge (stiff, nu, p1, p2, p3, xi_1, xi_2, xi_3, s_1, s_2, s_3, tau1_0, tau2_0, tau3_0, A, ls, ...
    init_ts, init_cs, init_fs, init_xis);
    [E, gradE, hessE_FDM] = Eb_gradEb_hessEb_FDM (stiff, nu, p1, p2, p3, xi_1, xi_2, xi_3, s_1, s_2, s_3, tau1_0, tau2_0, tau3_0, A, ls, ...
    init_ts, init_cs, init_fs, init_xis);


%% FDM check for gradient and hessian
% change = 1e-8;
% q = [p1; p2; p3; xi_1; xi_2; xi_3]; % dof vector
% 
% hessE_FDM = zeros(12,12);
% gradE_FDM = zeros(1,12);
% 
% for c = 1:12
%     q_change = q;
%     q_change(c) = q(c) + change;
%     p1_change = q_change(1:3);
%     p2_change = q_change(4:6);
%     p3_change = q_change(7:9);
%     xi_1_change = q_change(10);
%     xi_2_change = q_change(11);
%     xi_3_change = q_change(12);
% 
%     % changes in the energy
%     [E_change, gradE_change] = ...
%         Energy_Grad_Hess_with2terms_nat_curv ...
%         (stiff, nu, p1_change, p2_change, p3_change, xi_1_change, xi_2_change, xi_3_change, s_1, s_2, s_3, tau1_0, tau2_0, tau3_0, ...
%         init_ts, init_cs, init_fs, init_xis, t1, t2, t3, c1, c2, c3, A); % assume that t1, t2, t3, c1, c2, c3 are not changed - negligible changes
% 
%     gradE_FDM(c) = (E_change - E)/change;
% 
%     hessE_FDM(c,:) = (gradE_change - gradE) .* (1/change);
% 
% end
% diff_grad = gradE_FDM - gradE
diff_hess = hessE_FDM - hessE

%%

h1 = figure();
h1.WindowState = 'maximized';
% subplot(2,1,1);
% plot( gradE_with_stiff, 'ro');
% hold on
% plot( gradE_FDM, 'b^');
% hold off
% legend('Analytical my', 'Finite Difference');
% xlabel('Index');
% ylabel('Gradient');

subplot(2,1,2);
plot( reshape(hessE, [144,1]), 'ro');
hold on
plot( reshape(hessE_FDM, [144,1]), 'b^');
hold off
legend('Analytical', 'Finite Difference');
xlabel('Index');
ylabel('Hessian');