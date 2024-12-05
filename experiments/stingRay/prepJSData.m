
df = load('rawDataShell.txt');

nTri = MultiRod.n_faces;

scaleFactor = 100;

fileID = fopen('stingRayDataNewest.js', 'w');

fprintf(fileID, 'nTri = %i; \n', nTri);
fprintf(fileID, 'nodes = [ \n');

for i = 1:length(df)
    fprintf(fileID, '%f, %f, %f, \n', df(i,:)*scaleFactor);   
end
fprintf(fileID, ']; \n');
fclose(fileID);