% create input .txt file for shell
clc
clear all
close all

% rod nodes
nodes_rod = 49;
% nodes_rod = 0;

% Geometry
aspectRatio = 4; % Length/Width
geometry_w = 0.025; % width (m)
geometry_L = aspectRatio * geometry_w; % length (m)
maxMeshSize = geometry_w/4;
% maxMeshSize = 0.2;

%% Generate mesh and initialization

[Nodes, Edges, face_nodes, face_edges, sign_faces, EdgeIsBetween, HingeIsBetween, face_unit_normals, n_avg, edge_faces] =...
    generateMesh(geometry_L, geometry_w, maxMeshSize);


% [Nodes, Edges, face_nodes, face_edges, sign_faces, EdgeIsBetween, HingeIsBetween, face_unit_normals, n_avg] =...
%     generateMeshNewHemiSphere (geometry_L, geometry_w, 0.3);

% [Nodes, Edges, face_nodes, face_edges, sign_faces, EdgeIsBetween, HingeIsBetween, face_unit_normals, n_avg] =...
%     generateMeshNewSemiCyl (geometry_L, geometry_w, maxMeshSize);

n_nodes= size(Nodes,2);
n_edges = size(EdgeIsBetween,2);
n_elements = size(face_nodes,2);
n_hinges=size(HingeIsBetween,2);

% Nodes = [Nodes(1,:); Nodes(2,:) + 0.0125.*ones(1,size(Nodes(2,:),2)); Nodes(3,:)]; % for cylinder

%% for rod-shell
% face_nodes = face_nodes + nodes_rod.*ones(size(face_nodes));
% EdgeIsBetween = EdgeIsBetween + nodes_rod.*ones(size(EdgeIsBetween)); 
% HingeIsBetween = HingeIsBetween + nodes_rod.*ones(size(HingeIsBetween));
% 
% for i=1:n_elements
%     for j=1:3
%         if (face_nodes(j,i)==50) 
%             face_nodes(j,i)=1;
%         end
%     end
% end
% 
% for i=1:n_hinges
%     for j=1:4
%         if (HingeIsBetween(j,i)==50) 
%             HingeIsBetween(j,i)=1;
%         end
%     end
% end
% 
% for i=1:n_edges
%     for j=1:2
%         if (EdgeIsBetween(j,i)==50) 
%             EdgeIsBetween(j,i)=1;
%         end
%     end
% end
%% for inverted hemisphere
% Nodes = [Nodes(1,:); Nodes(2,:); -1.*Nodes(3,:)];
%%
% filename = 'input_shell_cantilever_most_dense.txt';
% filename = 'input_shell_cantilever_new.txt';
filename = 'input_shell_cantilever.txt';
% filename = 'new_input_shell.txt';
% filename = 'input semi-cylinder.txt';
% filename = 'input hemisphere.txt';
% filename = 'input hemisphere for c++ less dense.txt';

fid = fopen(filename,'w');
if fid ~= -1
    fprintf(fid,'*Nodes');
    fclose(fid);
end
writematrix(Nodes',filename, 'WriteMode','append')

fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*FaceNodes');
    fclose(fid);
end
writematrix(face_nodes',filename, 'WriteMode','append')

fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*Edges');
    fclose(fid);
end
writematrix(EdgeIsBetween',filename, 'WriteMode','append')

fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*Hinges');
    fclose(fid);
end
writematrix(HingeIsBetween',filename, 'WriteMode','append')





