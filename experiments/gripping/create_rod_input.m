% create straight inclined rod input file
n_nodes = 49;
start = [0,0,0];
last = [0.48,0,0];
nodes = [linspace(start(1),last(1),n_nodes)' linspace(start(2),last(2),n_nodes)' linspace(start(3),last(3),n_nodes)' ];
edges = zeros(n_nodes-1,2);
for i=1:n_nodes-1
    edges(i,:) = [i,i+1];
end

nodes = [nodes; 0.36, 0.01, 0.02; 0.36, -0.01, 0.02];

edges = [edges; n_nodes+1 n_nodes+2];

filename = 'gripping_rod_n49.txt';
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