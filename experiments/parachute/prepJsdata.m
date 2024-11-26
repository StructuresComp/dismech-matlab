% parachute : rod+shell
%% rod
df = load('rawDataRod.txt');

n_rod_nodes = MultiRod.n_rod_nodes;
n_Tri = MultiRod.n_faces;

rod_radius = 0.32;

scaleFactor = 100;

fileID = fopen('rodData.js', 'w');

fprintf(fileID, 'var rodData = { \n');
fprintf(fileID, 'nNodes : %i, \n', n_rod_nodes);
fprintf(fileID, 'rodRadius : %d,\n', rod_radius);
fprintf(fileID, 'nodesRod : [ \n');

for i = 1:length(df)
    t = df(i,1);
    r = df(i,2);
    x = df(i,3)*scaleFactor;
    y = df(i,4)*scaleFactor;
    z = df(i,5)*scaleFactor;
    fprintf(fileID, '%f, %i, %f, %f, %f, \n', t,r,x,y,z);   
end
fprintf(fileID, '] \n');
fprintf(fileID, '}; \n');
fclose(fileID);

%% shell 
ds = load('rawDataShell.txt');
shell_fileID = fopen('shellData.js', 'w');

fprintf(fileID, 'var shellData = { \n');
fprintf(fileID, 'nTri : %i, \n', n_Tri);
fprintf(fileID, 'nodes : [ \n');

for i = 1:length(ds)
    fprintf(fileID, '%f, %f, %f, \n', ds(i,:)*scaleFactor);   
end
fprintf(fileID, '] \n');
fprintf(fileID, '}; \n');
fclose(fileID);