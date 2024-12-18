% create straight inclined rod input file
n_nodes = 21;
start = [0,0,0];
last = [0.1,0,0];
nodes = [linspace(start(1),last(1),n_nodes)' linspace(start(2),last(2),n_nodes)' linspace(start(3),last(3),n_nodes)' ];
edges = zeros(n_nodes-1,2);
for i=1:n_nodes-1
    edges(i,:) = [i,i+1];
end
filename = 'horizontal_rod_n21.txt';
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