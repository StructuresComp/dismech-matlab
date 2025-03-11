clc
clear all
% close all
mesh_dense = [4,8, 20,25,30,35,40,45,50,55,60,65];
mesh_types = ["equilateral", "random", "right isoceles", "equilateral aligned", "non uniform"]; % type of mesh
%% Geometry L = 0.1 m, w = 0.02 m

for i=7
    maxMeshSize_factor_of_L = mesh_dense(i); % change this for different mesh densities 
    % and remember to mention in inputfile name that's generated

    for j=1:5
        meshType = mesh_types(j);
        if (meshType == "equilateral")
            geometry_w = 0.02166; % for equilateral (the number 0.02165 is due to the factor of sqrt(3)/2)
%             geometry_w = 0.02;
            geometry_L = 0.1;
            maxMeshSize = geometry_L/maxMeshSize_factor_of_L;
            filename = ['equilateral_mesh_' num2str(mesh_dense(i)) '.txt'];

        else   
            geometry_w = 0.02; % for random and right isoceles
            geometry_L = 0.1;
            maxMeshSize = geometry_L/maxMeshSize_factor_of_L;

            if(meshType == "random")
                filename = ['random_mesh_' num2str(mesh_dense(i)) '.txt'];
            elseif(meshType == "right isoceles")
                filename = ['right_mesh_' num2str(mesh_dense(i)) '.txt'];
            elseif (meshType == "equilateral aligned")
                filename = ['eq_algn_mesh_' num2str(mesh_dense(i)) '.txt'];
            elseif (meshType == "non uniform")
                filename = ['non_uniform_mesh_' num2str(mesh_dense(i)) '.txt'];
            end
        end

        %% Generate mesh and initialization
        [Nodes, face_nodes] =...
            generateTriangleMesh(geometry_L, geometry_w, maxMeshSize, meshType);

        fid = fopen(filename,'w');
        if fid ~= -1
            fprintf(fid,'*Nodes');
            fclose(fid);
        end
        writematrix(Nodes',filename, 'WriteMode','append')
        fid = fopen(filename, 'at');
        if fid ~= -1
            fprintf(fid,'*Triangles');
            fclose(fid);
        end
        writematrix(face_nodes',filename, 'WriteMode','append')

    end
end



