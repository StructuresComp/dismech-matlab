clc
clear all
% close all
mesh_dense = [4, 20,25,30,35,40,45,50,55,60,65];
mesh_types = ["equilateral", "random", "right isoceles", "equilateral aligned"]; % type of mesh
%% Geometry L = 0.1 m, w = 0.02 m

for i=1
    maxMeshSize_factor_of_L = mesh_dense(i); % change this for different mesh densities 
    % and remember to mention in inputfile name that's generated

    for j=1:4
        meshType = mesh_types(j);
        if (meshType == "equilateral")
            geometry_w = 0.1; % for equilateral (the number 0.02165 is due to the factor of sqrt(3)/2)
            geometry_L = 0.1;
            maxMeshSize = geometry_L/maxMeshSize_factor_of_L;
            filename = ['equilateral_mesh_' num2str(mesh_dense(i)) '.txt'];

        elseif( meshType == "random" || meshType == "right isoceles")   
            geometry_w = 0.1; % for random and right isoceles
            geometry_L = 0.1;
            maxMeshSize = geometry_L/maxMeshSize_factor_of_L;

            if(meshType == "random")
                filename = ['random_mesh_' num2str(mesh_dense(i)) '.txt'];
            else
                filename = ['right_mesh_' num2str(mesh_dense(i)) '.txt'];
            end
            
        elseif (meshType == "equilateral aligned")
            how_many = maxMeshSize_factor_of_L;
            geometry_L = 0.1;
            maxMeshSize = geometry_L*2/(sqrt(3)*how_many);
            geometry_w = 0.1;
            filename = ['eq_algn_mesh_' num2str(mesh_dense(i)) '.txt'];
        else 
            error ("select the meshType out of: random, equilateral, right isoceles, equilateral aligned");
        end

        %% Generate mesh and initialization
        [Nodes, Edges, face_nodes, face_edges, sign_faces, EdgeIsBetween, HingeIsBetween, face_unit_normals, n_avg, EdgeFaces] =...
            generateTriangleMesh(geometry_L, geometry_w, maxMeshSize, meshType);

        fid = fopen(filename,'w');
        if fid ~= -1
            fprintf(fid,'*shellNodes');
            fclose(fid);
        end
        writematrix(Nodes',filename, 'WriteMode','append')
        fid = fopen(filename, 'at');
        if fid ~= -1
            fprintf(fid,'*FaceNodes');
            fclose(fid);
        end
        writematrix(face_nodes',filename, 'WriteMode','append')

    end
end



