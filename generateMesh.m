function [Nodes, Final_Edges, Face_Nodes, Face_Edges, sign_faces, Final_EdgeIsBetween, Final_HingeIsBetween,...
    face_unit_norms, final_edge_avg_normal, final_edge_faces] ...
    = generateMesh(l, w, maxMeshSize)

% *************************************************************************
% Inputs:
% ----------------------------
% l : length of the rectangle
% w : width of the rectangle
% maxMeshSize : maximum size of the mesh
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
% As : Area of each face = 1*(no. of triangular faces)
% *************************************************************************
%% Meshing

minMeshSize = maxMeshSize/2; % minimum size of the mesh

gd = [3; 4; 0; l; l; 0; 0; 0; w; w];
g = decsg(gd);
model = createpde;
geometryFromEdges(model,g); % create geometry (rectangle in our case)

% FEMesh = generateMesh(model);
FEMesh = generateMesh(model,'Hmax', maxMeshSize, 'Hmin', ...
     minMeshSize, 'GeometricOrder', 'linear'); % generate the mesh

pdeplot(model)

% Extract the elements and nodes
Face_Nodes = FEMesh.Elements;
Nodes = FEMesh.Nodes;
Nodes = [Nodes; zeros(1, size(Nodes,2))];

%%
% no. of elements and nodes
n_nodes = size(Nodes,2);
numElements = size(Face_Nodes,2);

%% 
% Each elements is a triangle -> 
% node'k'_number gives the node id no. which is
% invloved in making the triangle
% so we can define edges using elements
% for each element - three edges
%     edge1 -> between node1 and node2 
%     edge2 -> between node2 and node3
%     edge3 -> between node3 and node1
% 
% but edges are common between elements, how to handle this? 
% and how do we no. these edges?

Edges=zeros(3,3*numElements); % actual no. of edges is less than 3*numElements
edge_index=1;
EdgeIsBetween=zeros(2,size(Edges,2));
hinge_index=1;
HingeIsBetween=zeros(4,size(Edges,2));
third_node=zeros(size(Edges,2),1);

Edge_Faces = zeros(2,size(Edges,2));

Face_Edges = zeros(3,numElements);
As = zeros(numElements,1);
sign_faces = zeros(3,numElements);
face_unit_norms = zeros(3,numElements);
edge_avg_normal = zeros(3,size(Edges,2));

% initializing some variables
Locb1_between = 0;
Locb2_between = 0;
Locb3_between = 0;

for c=1:numElements
    node1_number = Face_Nodes(1,c);
    node2_number = Face_Nodes(2,c);
    node3_number = Face_Nodes(3,c);
    node1_position = Nodes(:, node1_number);
    node2_position = Nodes(:, node2_number);
    node3_position = Nodes(:, node3_number);
    
    % face normal calculation:
    face_normal = cross(([node2_position]-[node1_position]),([node3_position]-[node1_position]));
    As(c) = norm(face_normal)/2;
    face_unit_norms(:,c) = face_normal .* 1/norm(face_normal);

    edge1=node3_position - node2_position;
    edge2=node1_position - node3_position;
    edge3=node2_position - node1_position;

    edge1_between=[node2_number; node3_number];
    edge1_between_negative=[node3_number; node2_number];
    
    edge2_between=[node3_number; node1_number];
    edge2_between_negative=[node1_number; node3_number];
    
    edge3_between=[node1_number; node2_number];
    edge3_between_negative=[node2_number; node1_number];

    % if the newly found edges are not counted already, add them to the Edges
    % array
    % else if it is already counted, it means that is a hinge! Handle
    % separately
    % find the '2' triangle elements corresponding to the edge and add the
    % node not corresponsing to the common edge to the hinge information!

    %% First Edge

    bool_edge1_between_absent=1;
    bool_edge1_between_negative_absent=1;
    edge1_between_present=(sum(ismember(edge1_between',EdgeIsBetween',"rows")));
    if(edge1_between_present>0) 
        bool_edge1_between_absent=0;
        [~,Locb1_between]=(ismember(edge1_between',EdgeIsBetween',"rows"));
    end
    edge1_between_negative_present=(sum(ismember(edge1_between_negative',EdgeIsBetween',"rows")));
    if(edge1_between_negative_present>0) 
        bool_edge1_between_negative_absent=0;
        [~,Locb1_between]=(ismember(edge1_between_negative',EdgeIsBetween',"rows"));
    end
    
    if(bool_edge1_between_absent && bool_edge1_between_negative_absent) % its an edge not hinge yet
        % add the edge to the Edges array
        Edges(:,edge_index)=edge1;
        % add the nodes of the corresponding edge to the EdgeIsBetween array
        EdgeIsBetween(:,edge_index)=[node2_number; node3_number];
        third_node(edge_index)=node1_number; % will be used for hinges

        % also map edge to face 
        Face_Edges(1,c) = edge_index;

        sign_faces(1,c) = 1;

        % edge average normal
        edge_avg_normal (:,edge_index) = face_unit_norms(:,c);

        % edge_faces
        Edge_Faces(:,edge_index) = [c;c];

        % update the edge_index counter
        edge_index=edge_index+1;
%         end
    else % it is a hinge
        oldnodes12=EdgeIsBetween(:,Locb1_between);
        third_node_old=third_node(Locb1_between);
        third_node_new=node1_number;
        HingeIsBetween(:,hinge_index)=[node2_number; node3_number; third_node_old; third_node_new];
        hinge_index=hinge_index+1;

        % also map edge to face 
        Face_Edges(1,c) = Locb1_between;

        % sign
        if(~bool_edge1_between_absent)
            sign_faces(1,c) = 1;
        elseif(~bool_edge1_between_negative_absent)
            sign_faces(1,c) = -1;
        else
            error('error in edge sign finding')
        end

        % edge average normal (for hinge case)
        edge_avg_normal(:,Locb1_between) = (edge_avg_normal(:,Locb1_between) + face_unit_norms(:,c))*0.5;

        % edge_faces
        Edge_Faces(2,Locb1_between) = c;

    end

% %       Method of finding the old third node by searching among the elements
% % __ Much more complex
% 
%         temp=zeros(3,1);
%         [old1_row,old1_column]=find(oldnodes12(1),Elements);
%         
%         for iter_elem=1:size(old1_column)
%             for i=1:3
%                 if(Elements(i,old1_column(iter_elem))==oldnodes12(2))
%                     temp(old1_row(iter_elem))=oldnodes12(1);
%                     temp(i)=oldnodes12(2);
%                     [~,~,thirdNodeOld]=find(Elements(:,old1_column(iter_elem))-temp)
%         
%                 end
%             end
%         end


    %% Second edge
    
    bool_edge2_between_absent=1;
    bool_edge2_between_negative_absent=1;
    edge2_between_present=(sum(ismember(edge2_between',EdgeIsBetween', "rows")));
    if(edge2_between_present>0) 
        bool_edge2_between_absent=0;
        [~,Locb2_between]=(ismember(edge2_between',EdgeIsBetween',"rows"));
    end
    
    edge2_between_negative_present=(sum(ismember(edge2_between_negative',EdgeIsBetween',"rows")));
    if(edge2_between_negative_present>0) 
        bool_edge2_between_negative_absent=0;
        [~,Locb2_between]=(ismember(edge2_between_negative',EdgeIsBetween',"rows"));
    end

    if(bool_edge2_between_absent && bool_edge2_between_negative_absent)

        % add the edge to the Edges array
        Edges(:,edge_index)=edge2;
         % add the nodes of the corresponding edge to the EdgeIsBetween
        % array
        EdgeIsBetween(:,edge_index)=[node3_number; node1_number];

        third_node(edge_index)=node2_number; % will be used for hinges

        % also map edge to face 
        Face_Edges(2,c) = edge_index;

        sign_faces(2,c) = 1;

        % edge average normal
        edge_avg_normal (:,edge_index) = face_unit_norms(:,c);

        % edge_faces
        Edge_Faces(:,edge_index) = [c;c];

        % update the edge_index counter
        edge_index=edge_index+1;

    else
        oldnodes23=EdgeIsBetween(:,Locb2_between);
        third_node_old=third_node(Locb2_between);
        third_node_new=node2_number;
        HingeIsBetween(:,hinge_index)=[node3_number; node1_number; third_node_old; third_node_new];
        hinge_index=hinge_index+1;

        % also map edge to face 
        Face_Edges(2,c) = Locb2_between;

        if(~bool_edge2_between_absent)
            sign_faces(2,c) = 1;
        elseif(~bool_edge2_between_negative_absent)
            sign_faces(2,c) = -1;
        else
            error('error in edge sign finding')
        end

        % edge average normal (for hinge case)
        edge_avg_normal(:,Locb2_between) = (edge_avg_normal(:,Locb2_between) + face_unit_norms(:,c))*0.5;

        % edge_faces
        Edge_Faces(2,Locb2_between) = c;
   
    end

    %% Third edge
    
    bool_edge3_between_absent=1;
    bool_edge3_between_negative_absent=1;
    edge3_between_present=(sum(ismember(edge3_between',EdgeIsBetween',"rows")));
    if(edge3_between_present>0) 
        bool_edge3_between_absent=0;
        [~,Locb3_between]=(ismember(edge3_between',EdgeIsBetween',"rows"));
    end
    
    edge3_between_negative_present=(sum(ismember(edge3_between_negative',EdgeIsBetween',"rows")));
    if(edge3_between_negative_present>0) 
        bool_edge3_between_negative_absent=0;
        [~,Locb3_between]=(ismember(edge3_between_negative',EdgeIsBetween',"rows"));
    end

    if(bool_edge3_between_absent && bool_edge3_between_negative_absent)

        % add the edge to the Edges array
        Edges(:,edge_index)=edge3;

        % add the nodes of the corresponding edge to the EdgeIsBetween array
        EdgeIsBetween(:,edge_index)=[node1_number; node2_number];

        third_node(edge_index)=node3_number; % will be used for hinges

        % also map edge to face 
        Face_Edges(3,c) = edge_index;

        sign_faces(3,c) = 1;

        % edge average normal
        edge_avg_normal (:,edge_index) = face_unit_norms(:,c);

        % edge_faces
        Edge_Faces(:,edge_index) = [c;c];

        % update the edge_index counter
        edge_index=edge_index+1;
        
    else
        oldnodes31=EdgeIsBetween(:,Locb3_between);
        third_node_old=third_node(Locb3_between);
        third_node_new=node3_number;
        HingeIsBetween(:,hinge_index)=[node1_number; node2_number; third_node_old; third_node_new];
        hinge_index=hinge_index+1;

        % also map edge to face 
        Face_Edges(3,c) = Locb3_between;

        if(~bool_edge3_between_absent)
            sign_faces(3,c) = 1;
        elseif(~bool_edge3_between_negative_absent)
            sign_faces(3,c) = -1;
        else
            error('error in edge sign finding')
        end

        % edge average normal (for hinge case)
        edge_avg_normal(:,Locb3_between) = (edge_avg_normal(:,Locb3_between) + face_unit_norms(:,c))*0.5;

        % edge_faces
        Edge_Faces(2,Locb3_between) = c;

    end

end
Actual_n_edges = edge_index-1;
Final_Edges = Edges(:,1:Actual_n_edges);
Final_EdgeIsBetween = EdgeIsBetween(:,1:Actual_n_edges);

Actual_n_hinges = hinge_index-1;
Final_HingeIsBetween = HingeIsBetween(:,1:Actual_n_hinges);

final_edge_avg_normal = edge_avg_normal(:,1:Actual_n_edges);

final_edge_faces = Edge_Faces(:,1:Actual_n_edges);
end
