function [Nodes, Face_Nodes] ...
    = generateTriangleMesh(l, w, maxMeshSize, meshType, minMeshSize)
if (nargin<5)
    minMeshSize = maxMeshSize*0.75;
end

% *************************************************************************
% Inputs:
% ----------------------------
% l : length of the rectangle
% w : width of the rectangle
% maxMeshSize : maximum size of the mesh
% meshType : denotes the shape of mesh triangles- amongst "equilateral",
% "random", "right isoceles", "equilateral aligned"
%
% Outputs:
% --------
% Nodes : Node position vectors = 3*(no. of nodes)
% Final_Edges : Edge vectors = 3*(no. of edges)
% Face_Nodes : node indices corresponding to each face = 3*(no. of triangular faces)
% Face_Edges : edge indices corresponding to each face = 3*(no. of triangular faces)
% sign_faces : sign value (-1/+1) corresponding to ownership of the edge
% Final_EdgeIsBetween : node indices corresponding to each edge = 2*(no. of edges)
% Final_HingeIsBetween : node indices corresponding to each hinge = 4*(no. of hinges)
% final_Edge_Faces : face indices corresponding to each edge = 2*(no. of edges)
% face_unit_norms : unit normal vectors of each face = 3*(no. of triangular faces)
% final_edge_avg_normal : edge average normal vectors (unit vectors) = 3*(no. of edges)
% final_edge_faces : face indices corresponding to each edge (2 if hinge, 1 if simply edge) = 2*(no. of edges)
% As : Area of each face = 1*(no. of triangular faces)
% *************************************************************************
%% Meshing

if (meshType == "random")

    minMeshSize = maxMeshSize/2; % minimum size of the mesh
    
    gd = [3; 4; 0; l; l; 0; 0; 0; w; w];
    g = decsg(gd);
    model = createpde;
    geometryFromEdges(model,g); % create geometry (rectangle in our case)
    
    % FEMesh = generateMesh(model);
    FEMesh = generateMesh(model,'Hmax', maxMeshSize, 'Hmin', ...
         minMeshSize, 'GeometricOrder', 'linear'); % generate the mesh

    figure(1)
    pdeplot(model)
    axis equal;
    title(meshType)

    % Extract the elements and nodes
    Face_Nodes = FEMesh.Elements;
    Nodes = FEMesh.Nodes;

elseif (meshType == "equilateral")
    n_ = round(l/maxMeshSize);
    h0=l/n_;
    % row_wise_n = round(w/(h0*sqrt(3)/2));
    [x,y]=meshgrid(0:h0:l,0:h0*sqrt(3)/2:w);
    x(2:2:end,:)=x(2:2:end,:)+h0/2; % Shift even rows

    % Flatten the arrays
    x = x(:);
    y = y(:);
    
    % Remove points outside the rectangular domain (x > l or y > w)
    valid_idx = (x <= l) & (y <= w);
    x = x(valid_idx);
    y = y(valid_idx);

    DT=delaunayTriangulation(x(:),y(:));

    figure(2)
    triplot(DT);
    axis equal;
    title(meshType)
    
    % Extract the elements and nodes
    Face_Nodes = DT.ConnectivityList';
    Nodes = DT.Points';

elseif (meshType =="right isoceles")
    small_geom_tol = w*0.05;
    n_ = round(l/maxMeshSize);
    h0=l/n_;
    along_l = 0:h0:l;
    along_w = 0:h0:w;
    if ( (w>along_w(end)+small_geom_tol) || (w<along_w(end)-small_geom_tol))
        n_w = round(w/maxMeshSize);
        if(n_w==0)
            n_w = n_w+1;
        end
        along_w = 0:w/n_w:w;
%         along_w = [along_w, w];
    end
    [x,y]=meshgrid(along_l,along_w);
    DT=delaunayTriangulation(x(:),y(:));

    figure(3)
    triplot(DT);
    axis equal;
    title(meshType)
    
    % Extract the elements and nodes
    Face_Nodes = DT.ConnectivityList';
    Nodes = DT.Points';

elseif (meshType == "equilateral aligned")
    n_ = round(l/(sqrt(3)/2*maxMeshSize));
    h0=l/n_;
    n_w = round(w/maxMeshSize);
    w0=w/n_w;
    [x,y]=meshgrid(linspace(0,l,n_+1),0:w0:w);
    y(:,2:2:end)=y(:,2:2:end)+w0/2; % Shift even rows
    % Flatten arrays
    x = x(:); y = y(:);
    % Remove points outside the domain (y > w or x > l)
    valid_idx = (y <= w) & (x <= l);
    x = x(valid_idx);
    y = y(valid_idx);

    DT=delaunayTriangulation(x(:),y(:));
    
    figure(4)
    triplot(DT);
    axis equal;
    title(meshType)

    % Extract the elements and nodes
    Face_Nodes = DT.ConnectivityList';
    Nodes = DT.Points';

elseif (meshType == "non uniform")
    DT = nonUniformMesh(l, w, minMeshSize, maxMeshSize);
    % Extract the elements and nodes
    Face_Nodes = DT.ConnectivityList';
    Nodes = DT.Points';

    % Plot the mesh
    figure(5);
    triplot(DT);
    axis equal;
    title('Non-Uniform Mesh');
    xlabel('X');
    ylabel('Y');

else 
    error ("select the meshType out of: random, equilateral, right isoceles, equilateral aligned");
end

Nodes = [Nodes; zeros(1, size(Nodes,2))];

end

