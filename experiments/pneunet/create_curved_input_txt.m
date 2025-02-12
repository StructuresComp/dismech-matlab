% create straight vertical input file
% create curved rod input file
n_nodes = 51;
l = 0.1;
dl = l/(n_nodes-1);
nodes = zeros(n_nodes,3);
for i=1:n_nodes
    nodes(i,1) = (i-1)*dl;
    nodes(i,2) = 0.0;
    nodes(i,3) = 0.0;
end
edges = zeros(n_nodes-1,2);
for i=1:n_nodes-1
    edges(i,:) = [i,i+1];
end
filename = 'input_straight_horizontal_shorter.txt';
fid = fopen(filename,'w');
if fid ~= -1
    fprintf(fid,'*Nodes');
    fclose(fid);
end
writematrix(nodes,filename, 'WriteMode','append')
fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*Edges');
    fclose(fid);
end
writematrix(edges,filename, 'WriteMode','append')


%% create curved rod input file
% n_nodes = 51;
% l = 1;
% r = l/pi;
% dtheta = 180/50;
% theta = 0;
% nodes = zeros(n_nodes,3);
% for i=1:n_nodes
%     nodes(i,1) = r*sind(theta);
%     nodes(i,2) = 0;
%     nodes(i,3) = -r+r*cosd(theta);
%     theta = theta + dtheta;
% end
% nodes(end,1) = 0;
% edges = zeros(n_nodes-1,2);
% for i=1:n_nodes-1
%     edges(i,:) = [i,i+1];
% end
% filename = 'input_semicircle.txt';
% fid = fopen(filename,'w');
% if fid ~= -1
%     fprintf(fid,'*rodNodes');
%     fclose(fid);
% end
% writematrix(nodes,filename, 'WriteMode','append')
% fid = fopen(filename, 'at');
% if fid ~= -1
%     fprintf(fid,'*rodEdges');
%     fclose(fid);
% end
% writematrix(edges,filename, 'WriteMode','append')
% 

