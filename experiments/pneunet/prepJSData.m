
df = load('rawDataRod.txt');

n_nodes = MultiRod.n_nodes;
rod_radius = 0.32;

scaleFactor = 10;

fileID = fopen('rodData3.js', 'w');

fprintf(fileID, 'nNodes = %i; \n', n_nodes);
fprintf(fileID, 'rodRadius=%d;\n', rod_radius);
fprintf(fileID, 'nodesRod = [ \n');

for i = 1:length(df)
    t = df(i,1);
    x = df(i,2)*-scaleFactor;
    y = df(i,3)*scaleFactor;
    z = df(i,4)*scaleFactor;
    fprintf(fileID, '%f, 0, %f, %f, %f, \n', t,x,y,z);   
end
fprintf(fileID, ']; \n');
fclose(fileID);