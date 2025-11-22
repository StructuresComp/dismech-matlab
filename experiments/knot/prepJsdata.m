% parachute : rod+shell
%% rod
df = load('rawDataRod.txt');

n_rod_nodes = 301;
n_Tri = 0;

rod_radius = 0.04;

scaleFactor = 10;

fileID = fopen('rodData_knot_new_matlab.js', 'w');

fprintf(fileID, 'nNodes = %i; \n', n_rod_nodes);
fprintf(fileID, 'rodRadius = %d;\n', rod_radius);
fprintf(fileID, 'nodesRod = [ \n');

for i = 1:length(df)
    t = df(i,1);
    x = df(i,2)*scaleFactor;
    y = df(i,3)*scaleFactor;
    z = df(i,4)*scaleFactor;
    fprintf(fileID, '%f, %i, %f, %f, %f, \n', t,1,x,y,z);   
end
fprintf(fileID, '] \n');
fprintf(fileID, '; \n');
fclose(fileID);

%% shell 
% ds = load('rawDataShell.txt');
% shell_fileID = fopen('shellData.js', 'w');
% 
% fprintf(shell_fileID, 'var shellData = { \n');
% fprintf(shell_fileID, 'nTri : %i, \n', n_Tri);
% fprintf(shell_fileID, 'nodes : [ \n');
% 
% for i = 1:length(ds)
%     fprintf(shell_fileID, '%f, %f, %f, \n', ds(i,:)*scaleFactor);   
% end
% fprintf(shell_fileID, '] \n');
% fprintf(shell_fileID, '}; \n');
% fclose(shell_fileID);