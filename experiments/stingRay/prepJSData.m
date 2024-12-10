
df = load('rawDataShell.txt');

nTri = MultiRod.n_faces;

scaleFactor = 100;

fileID = fopen('stingRayData_n10_Y6e9_Cd0pt5_A2piby3_5s.js', 'w');

fprintf(fileID, 'nTri = %i; \n', nTri);
fprintf(fileID, 'nodes = [ \n');

for i = 1:length(df)
    fprintf(fileID, '%f, %f, %f, \n', df(i,:)*scaleFactor);   
end
fprintf(fileID, ']; \n');
fclose(fileID);