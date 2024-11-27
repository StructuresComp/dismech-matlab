
df = load('rawDataShell.txt');

nTri = softRobot.n_faces;

scaleFactor = 100;

FileName = strcat('plateData_midedge_', num2str(material.youngs_shell), '.js');


% fileID = fopen('plateData_midedge_2e8.js', 'w');
fileID = fopen(FileName, 'w');

fprintf(fileID, 'nTri = %i; \n', nTri);
fprintf(fileID, 'nodes = [ \n');

for i = 1:length(df)
    fprintf(fileID, '%f, %f, %f, \n', df(i,:)*scaleFactor);   
end
fprintf(fileID, ']; \n');
fclose(fileID);