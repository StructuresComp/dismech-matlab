clc
clear all
close all

n = 3;
L = 1; % side length of triangle
% centroid of triangle
c = [-L/2/sqrt(3), 0];
% rope1
rope1_s = [c,-0.1]';
rope1_e = [0,L/2,0]';
rope1_nodes = [linspace(rope1_s(1),rope1_e(1),n); linspace(rope1_s(2),rope1_e(2),n); linspace(rope1_s(3),rope1_e(3),n)]';
n_nodes_r1 = size(rope1_nodes,1) - 1;
rope1_edges = [1:n_nodes_r1-1; 2:n_nodes_r1]';

% rope2
rope2_s = rope1_s;
rope2_e = [0,-L/2,0]';
rope2_nodes = [linspace(rope2_s(1),rope2_e(1),n); linspace(rope2_s(2),rope2_e(2),n); linspace(rope2_s(3),rope2_e(3),n)]';
n_nodes_r2 = size(rope2_nodes,1) - 2;
rope2_edges = [1, n_nodes_r1 + 1 : n_nodes_r1 + n_nodes_r2-1; ...
    n_nodes_r1 + 1 , n_nodes_r1 + 2 : n_nodes_r1 + n_nodes_r2]';

% rope3
rope3_s = rope1_s;
rope3_e = [-L*sqrt(3)/2,0,0]';
rope3_nodes = [linspace(rope3_s(1),rope3_e(1),n); linspace(rope3_s(2),rope3_e(2),n); linspace(rope3_s(3),rope3_e(3),n)]';
n_nodes_r3 = size(rope3_nodes,1) - 2;
a = (n_nodes_r1 + n_nodes_r2 + 1) : (n_nodes_r1 + n_nodes_r2 + n_nodes_r3 - 1);
b = (n_nodes_r1 + n_nodes_r2 + 2) : (n_nodes_r1 + n_nodes_r2 + n_nodes_r3);
rope3_edges = [1, a; n_nodes_r1 + n_nodes_r2 + 1, b]';

rod_nodes = [rope1_nodes(1:end-1,:); rope2_nodes(2:end-1,:); rope3_nodes(2:end-1,:)];
rod_edges = [rope1_edges; rope2_edges; rope3_edges];
figure()
plot3(rod_nodes(:,1), rod_nodes(:,2), rod_nodes(:,3));

%% triangular shell
l = L/n;
y = [];
x = [];

for i = 0:1:n
    y_temp =  (-i*l/2:l:i*l/2);
     y = [y, y_temp];

     x_temp = (sqrt(3)/2)*(i-n)*l .*ones(1,size(y_temp,2));
     x = [x, x_temp];
end

figure()
scatter(x,y);

DT=delaunayTriangulation(x(:),y(:));
figure()
triplot(DT);
axis equal;

% % Extract the elements and nodes
% Face_Nodes = DT.ConnectivityList;
% Nodes = DT.Points;
% Nodes = [Nodes, zeros(size(Nodes,1),1)];

% DT=delaunay(x_total(:),y_total(:));
% Face_Nodes = DT;

% check face normals
face_normals = faceNormal(DT);

shell_nodes = DT.Points;
shell_nodes(abs(shell_nodes)<1e-8) = 0;
shell_nodes = [shell_nodes , zeros(size(shell_nodes,1),1)];
face_nodes = DT.ConnectivityList;
n_rod_nodes = size(rod_nodes,1);
face_nodes = face_nodes + n_rod_nodes.*ones(size(face_nodes,1),3);

Nodes = [rod_nodes;shell_nodes];

%% rod-shell joint edges

rod_shell_joint_edges = [];
% joint 1: rod1 last node and corresponding shell node
[~,ind1] = ismember([0,0.5,0],shell_nodes,'rows');
rod_shell_joint_edges = [rod_shell_joint_edges; n_nodes_r1 (ind1+n_rod_nodes)];

% joint 2: rod2 last node and corresponding shell node
[~,ind2] = ismember([0,-0.5,0],shell_nodes,'rows');
rod_shell_joint_edges = [rod_shell_joint_edges; (n_nodes_r1 + n_nodes_r2) (ind2+n_rod_nodes)];

% joint 3: rod3 last node and shell node 1
rod_shell_joint_edges = [rod_shell_joint_edges; (n_nodes_r1 + n_nodes_r2 + n_nodes_r3) (1+n_rod_nodes)];

%%
filename = 'triangle_parachute_n3_matlab.txt';
fid = fopen(filename,'w');
if fid ~= -1
    fprintf(fid,'*rodNodes');
    fclose(fid);
end
writematrix(rod_nodes,filename, 'WriteMode','append')
fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*shellNodes');
    fclose(fid);
end
writematrix(shell_nodes,filename, 'WriteMode','append')

fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*FaceNodes');
    fclose(fid);
end
writematrix(face_nodes,filename, 'WriteMode','append')

fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*rodEdges');
    fclose(fid);
end
writematrix(rod_edges,filename, 'WriteMode','append')

fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*rodshellJointEdges');
    fclose(fid);
end
writematrix(rod_shell_joint_edges,filename, 'WriteMode','append')

