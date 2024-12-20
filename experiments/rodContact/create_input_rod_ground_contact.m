% create straight inclined rod input file
n_nodes = 2;
% start = [0,0,0];
% last = [0.05,0.025,0.05];
start = [0,0,-0.05];
last = [0.05,0, -0.05];
nodes = [linspace(start(1),last(1),n_nodes)' linspace(start(2),last(2),n_nodes)' linspace(start(3),last(3),n_nodes)' ];
edges = zeros(n_nodes-1,2);
for i=1:n_nodes-1
    edges(i,:) = [i,i+1];
end
filename = 'input_straight_horizontal_smaller_n2.txt';
fid = fopen(filename,'w');
if fid ~= -1
    fprintf(fid,'*rodNodes');
    fclose(fid);
end
writematrix(nodes,filename, 'WriteMode','append')
fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*rodEdges');
    fclose(fid);
end
writematrix(edges,filename, 'WriteMode','append')

