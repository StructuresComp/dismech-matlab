% parachute : rod+shell
%% rod
% df = load('rawDataRod.txt');
scaleFactor = 10;

node_data = scaleFactor.*dof_with_time(2:3*MultiRod.n_nodes+1,:)';
connectivity_matrix = elStretchRod - int64(ones(size(elStretchRod)));

n_rod_nodes = MultiRod.n_rod_nodes;
n_Tri = MultiRod.n_faces;

rod_radius = 0.32;



fileID = fopen('rodData.js', 'w');

fprintf(fileID, 'var rodData = { \n');
% fprintf(fileID, 'nNodes : %i, \n', n_rod_nodes);
% fprintf(fileID, 'rodRadius : %d,\n', rod_radius);
fprintf(fileID, 'nodePositions : [ \n');

for i = 1:size(node_data, 1)
    % Convert the row into a comma-separated string enclosed in square brackets
    rowString = sprintf('[%s],', strjoin(string(node_data(i, :)), ', '));
    
    % Write the rowString into the file
    fprintf(fileID, '%s\n', rowString);
end

fprintf(fileID, '] , \n');

fprintf(fileID, 'connectivity : [ \n');

for i = 1:size(connectivity_matrix, 1)
    % Convert the row into a comma-separated string enclosed in square brackets
    rowString = sprintf('[%s],', strjoin(string(connectivity_matrix(i, :)), ', '));
    
    % Write the rowString into the file
    fprintf(fileID, '%s\n', rowString);
end

fprintf(fileID, '] \n');

fprintf(fileID, '}; \n');
fclose(fileID);

%%
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
