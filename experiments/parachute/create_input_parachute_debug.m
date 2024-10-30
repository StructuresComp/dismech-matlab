clc
clear all
close all

%% make ropes
dl = 0.25;
% rope1
rope1_s = [0,0,-0.5]';
rope1_e = [0.5,0.5,0]';
rope1_nodes = [rope1_s(1):dl:rope1_e(1); rope1_s(2):dl:rope1_e(2); rope1_s(3):dl:rope1_e(3)]';
n_nodes_r1 = size(rope1_nodes,1) - 1;
rope1_edges = [1:n_nodes_r1-1; 2:n_nodes_r1]';

% rope2
rope2_s = rope1_s;
rope2_e = [0.5,-0.5,0]';
rope2_nodes = [rope2_s(1):dl:rope2_e(1); rope2_s(2):-dl:rope2_e(2); rope2_s(3):dl:rope2_e(3)]';
n_nodes_r2 = size(rope2_nodes,1) - 2;
rope2_edges = [1, n_nodes_r1 + 1 : n_nodes_r1 + n_nodes_r2-1; ...
    n_nodes_r1 + 1 , n_nodes_r1 + 2 : n_nodes_r1 + n_nodes_r2]';

% rope3
rope3_s = rope1_s;
rope3_e = [-0.5,-0.5,0]';
rope3_nodes = [rope3_s(1):-dl:rope3_e(1); rope3_s(2):-dl:rope3_e(2); rope3_s(3):dl:rope3_e(3)]';
n_nodes_r3 = size(rope3_nodes,1) - 2;
a = (n_nodes_r1 + n_nodes_r2 + 1) : (n_nodes_r1 + n_nodes_r2 + n_nodes_r3 - 1);
b = (n_nodes_r1 + n_nodes_r2 + 2) : (n_nodes_r1 + n_nodes_r2 + n_nodes_r3);
rope3_edges = [1, a; n_nodes_r1 + n_nodes_r2 + 1, b]';

% rope4
rope4_s = rope1_s;
rope4_e = [-0.5,0.5,0]';
rope4_nodes = [rope4_s(1):-dl:rope4_e(1); rope4_s(2):dl:rope4_e(2); rope4_s(3):dl:rope4_e(3)]';
n_nodes_r4 = size(rope4_nodes,1) - 2;
a = (n_nodes_r1 + n_nodes_r2 + n_nodes_r3 + 1) : (n_nodes_r1 + n_nodes_r2 + n_nodes_r3 + n_nodes_r4 -1);
b = (n_nodes_r1 + n_nodes_r2 + n_nodes_r3 + 2) : (n_nodes_r1 + n_nodes_r2 + n_nodes_r3 + n_nodes_r4);
rope4_edges = [1, a; n_nodes_r1 + n_nodes_r2 + n_nodes_r3 + 1, b]';

figure()
plot3(rope1_nodes(:,1), rope1_nodes(:,2), rope1_nodes(:,3));
hold on;
plot3(rope2_nodes(:,1), rope2_nodes(:,2), rope2_nodes(:,3),'r')
plot3(rope3_nodes(:,1), rope3_nodes(:,2), rope3_nodes(:,3),'k')
plot3(rope4_nodes(:,1), rope4_nodes(:,2), rope4_nodes(:,3),'m')
hold off;

rod_nodes = [rope1_nodes(1:end-1,:); rope2_nodes(2:end-1,:); rope3_nodes(2:end-1,:); rope4_nodes(2:end-1,:)];
rod_edges = [rope1_edges; rope2_edges; rope3_edges; rope4_edges];
figure()
plot3(rod_nodes(:,1), rod_nodes(:,2), rod_nodes(:,3));

%% canopy shell
maxMeshSize = dl*3;
minMeshSize = maxMeshSize/2; % minimum size of the mesh
    
gd = [3; 4; -0.5; 0.5; 0.5; -0.5; -0.5; -0.5; 0.5; 0.5];
g = decsg(gd);
model = createpde;
geometryFromEdges(model,g); % create geometry (rectangle in our case)

% FEMesh = generateMesh(model);
FEMesh = generateMesh(model,'Hmax', maxMeshSize, 'Hmin', ...
     minMeshSize, 'GeometricOrder', 'linear'); % generate the mesh
figure()
pdeplot(model)
axis equal;

shell_nodes = FEMesh.Nodes;
shell_nodes(abs(shell_nodes)<1e-8) = 0;
shell_nodes = [shell_nodes ; zeros(1,size(shell_nodes,2))]';
face_nodes = FEMesh.Elements';
n_rod_nodes = size(rod_nodes,1);
face_nodes = face_nodes + n_rod_nodes.*ones(size(face_nodes,1),3);

Nodes = [rod_nodes;shell_nodes];

%% rod-shell joint edges

rod_shell_joint_edges = [];
% joint 1: rod1 last node and shell node 3
rod_shell_joint_edges = [rod_shell_joint_edges; n_nodes_r1 (3+n_rod_nodes)];

% joint 2: rod2 last node and shell node 2
rod_shell_joint_edges = [rod_shell_joint_edges; (n_nodes_r1 + n_nodes_r2) (2+n_rod_nodes)];

% joint 3: rod3 last node and shell node 1
rod_shell_joint_edges = [rod_shell_joint_edges; (n_nodes_r1 + n_nodes_r2 + n_nodes_r3) (1+n_rod_nodes)];

% joint 4: rod2 last node and shell node 4
rod_shell_joint_edges = [rod_shell_joint_edges; (n_nodes_r1 + n_nodes_r2 + n_nodes_r3 + n_nodes_r4) (4+n_rod_nodes)];


%% write input
filename = 'parachute_input_dl25.txt';
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

