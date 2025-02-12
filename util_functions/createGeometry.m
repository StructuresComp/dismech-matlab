% Input:
% nodes,
% edges
% face_nodes

% Ouput:
% nodes, 
% Edges, 
% rod_edges, 
% shell_edges, 
% rod_shell_joint_edges, 
% rod_shell_joint_edges_total, 
% face_nodes, 
% face_edges,
% rod_stretch_springs, 
% shell_stretch_springs, 
% bend_twist_springs, 
% bend_twist_signs, 
% hinges, 
% sign_faces, 
% face_unit_norms

function [nodes, Edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_edges_total, face_nodes, face_edges, ...
    rod_stretch_springs, shell_stretch_springs, bend_twist_springs, bend_twist_signs, hinges, sign_faces, face_unit_norms] = createGeometry(nodes, edges, face_nodes)

% find the rod-shell joint edges
[rod_shell_joint_edges, rod_edges] = separate_joint_edges(face_nodes, edges);

n_nodes = size(nodes,1);
n_rod_edges = size(rod_edges,1);
n_rod_shell_joints = size(rod_shell_joint_edges,1);
n_edges = n_rod_edges + n_rod_shell_joints;
n_faces = size(face_nodes,1);
%% shell related computations
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

nEdges=3*n_faces; % actual no. of edges is less than 3*numElements
shell_edge_index=1;
shell_edges=int64(zeros(nEdges,2));
hinge_index=1;
hinges=zeros(nEdges,4);
third_node=zeros(nEdges,1);

Edge_Faces = zeros(nEdges,2);

face_shell_edges = zeros(n_faces,3);
As = zeros(n_faces,1);
sign_faces = zeros(n_faces,3);
face_unit_norms = zeros(3,n_faces);
% edge_avg_normal = zeros(3,Edges);

% initializing some variables
Locb1_between = 0;
Locb2_between = 0;
Locb3_between = 0;

for c=1:n_faces
    node1_number = face_nodes(c,1);
    node2_number = face_nodes(c,2);
    node3_number = face_nodes(c,3);
    node1_position = nodes(node1_number,:);
    node2_position = nodes(node2_number,:);
    node3_position = nodes(node3_number,:);
    
    % face normal calculation:
    face_normal = cross(([node2_position]-[node1_position]),([node3_position]-[node1_position]));
    As(c) = norm(face_normal)/2;
    face_unit_norms(:,c) = face_normal .* 1/norm(face_normal);

    edge1_between=[node2_number, node3_number];
    edge1_between_negative=[node3_number, node2_number];
    
    edge2_between=[node3_number, node1_number];
    edge2_between_negative=[node1_number, node3_number];
    
    edge3_between=[node1_number, node2_number];
    edge3_between_negative=[node2_number, node1_number];

    % if the newly found edges are not counted already, add them to the Edges
    % array
    % else if it is already counted, it means that is a hinge! Handle
    % separately
    % find the '2' triangle elements corresponding to the edge and add the
    % node not corresponsing to the common edge to the hinge information!

    %% First Edge

    bool_edge1_between_absent=1;
    bool_edge1_between_negative_absent=1;
    edge1_between_present=(sum(ismember(edge1_between,shell_edges,"rows")));
    if(edge1_between_present>0) 
        bool_edge1_between_absent=0;
        [~,Locb1_between]=(ismember(edge1_between,shell_edges,"rows"));
    end
    edge1_between_negative_present=(sum(ismember(edge1_between_negative,shell_edges,"rows")));
    if(edge1_between_negative_present>0) 
        bool_edge1_between_negative_absent=0;
        [~,Locb1_between]=(ismember(edge1_between_negative,shell_edges,"rows"));
    end
    
    if(bool_edge1_between_absent && bool_edge1_between_negative_absent) % its an edge not hinge yet
        % add the nodes of the corresponding edge to the EdgeIsBetween array
        shell_edges(shell_edge_index,:)=[node2_number, node3_number];
        third_node(shell_edge_index)=node1_number; % will be used for hinges

        % also map edge to face 
        face_shell_edges(c,1) = shell_edge_index;

        sign_faces(c,1) = 1;

        % edge average normal
%         edge_avg_normal (:,shell_edge_index) = face_unit_norms(:,c);

        % edge_faces
        Edge_Faces(shell_edge_index,:) = [c,c];

        % update the edge_index counter
        shell_edge_index=shell_edge_index+1;
%         end
    else % it is a hinge
        oldnodes12=shell_edges(Locb1_between,:);
        third_node_old=third_node(Locb1_between);
        third_node_new=node1_number;
        hinges(hinge_index,:)=[node2_number, node3_number, third_node_old, third_node_new];
        hinge_index=hinge_index+1;

        % also map edge to face 
        face_shell_edges(c,1) = Locb1_between;

        % sign
        if(~bool_edge1_between_absent)
            sign_faces(c,1) = 1;
        elseif(~bool_edge1_between_negative_absent)
            sign_faces(c,1) = -1;
        else
            error('error in edge sign finding')
        end

        % edge average normal (for hinge case)
%         edge_avg_normal(:,Locb1_between) = (edge_avg_normal(:,Locb1_between) + face_unit_norms(:,c))*0.5;

        % edge_faces
        Edge_Faces(Locb1_between,2) = c;

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
    edge2_between_present=(sum(ismember(edge2_between,shell_edges, "rows")));
    if(edge2_between_present>0) 
        bool_edge2_between_absent=0;
        [~,Locb2_between]=(ismember(edge2_between,shell_edges,"rows"));
    end
    
    edge2_between_negative_present=(sum(ismember(edge2_between_negative,shell_edges,"rows")));
    if(edge2_between_negative_present>0) 
        bool_edge2_between_negative_absent=0;
        [~,Locb2_between]=(ismember(edge2_between_negative,shell_edges,"rows"));
    end

    if(bool_edge2_between_absent && bool_edge2_between_negative_absent)

         % add the nodes of the corresponding edge to the EdgeIsBetween
        % array
        shell_edges(shell_edge_index,:)=[node3_number, node1_number];

        third_node(shell_edge_index)=node2_number; % will be used for hinges

        % also map edge to face 
        face_shell_edges(c,2) = shell_edge_index;

        sign_faces(c,2) = 1;

        % edge average normal
%         edge_avg_normal (:,shell_edge_index) = face_unit_norms(:,c);

        % edge_faces
        Edge_Faces(shell_edge_index,:) = [c,c];

        % update the edge_index counter
        shell_edge_index=shell_edge_index+1;

    else
        oldnodes23=shell_edges(Locb2_between,:);
        third_node_old=third_node(Locb2_between);
        third_node_new=node2_number;
        hinges(hinge_index,:)=[node3_number, node1_number, third_node_old, third_node_new];
        hinge_index=hinge_index+1;

        % also map edge to face 
        face_shell_edges(c,2) = Locb2_between;

        if(~bool_edge2_between_absent)
            sign_faces(c,2) = 1;
        elseif(~bool_edge2_between_negative_absent)
            sign_faces(c,2) = -1;
        else
            error('error in edge sign finding')
        end

        % edge average normal (for hinge case)
%         edge_avg_normal(:,Locb2_between) = (edge_avg_normal(:,Locb2_between) + face_unit_norms(:,c))*0.5;

        % edge_faces
        Edge_Faces(Locb2_between,2) = c;
   
    end

    %% Third edge
    
    bool_edge3_between_absent=1;
    bool_edge3_between_negative_absent=1;
    edge3_between_present=(sum(ismember(edge3_between,shell_edges,"rows")));
    if(edge3_between_present>0) 
        bool_edge3_between_absent=0;
        [~,Locb3_between]=(ismember(edge3_between,shell_edges,"rows"));
    end
    
    edge3_between_negative_present=(sum(ismember(edge3_between_negative,shell_edges,"rows")));
    if(edge3_between_negative_present>0) 
        bool_edge3_between_negative_absent=0;
        [~,Locb3_between]=(ismember(edge3_between_negative,shell_edges,"rows"));
    end

    if(bool_edge3_between_absent && bool_edge3_between_negative_absent)

        % add the nodes of the corresponding edge to the EdgeIsBetween array
        shell_edges(shell_edge_index,:)=[node1_number, node2_number];

        third_node(shell_edge_index)=node3_number; % will be used for hinges

        % also map edge to face 
        face_shell_edges(c,3) = shell_edge_index;

        sign_faces(c,3) = 1;

        % edge average normal
%         edge_avg_normal (:,shell_edge_index) = face_unit_norms(:,c);

        % edge_faces
        Edge_Faces(shell_edge_index,:) = [c,c];

        % update the edge_index counter
        shell_edge_index=shell_edge_index+1;
        
    else
        oldnodes31=shell_edges(Locb3_between,:);
        third_node_old=third_node(Locb3_between);
        third_node_new=node3_number;
        hinges(hinge_index,:)=[node1_number, node2_number, third_node_old, third_node_new];
        hinge_index=hinge_index+1;

        % also map edge to face 
        face_shell_edges(c,3) = Locb3_between;

        if(~bool_edge3_between_absent)
            sign_faces(c,3) = 1;
        elseif(~bool_edge3_between_negative_absent)
            sign_faces(c,3) = -1;
        else
            error('error in edge sign finding')
        end

        % edge average normal (for hinge case)
%         edge_avg_normal(:,Locb3_between) = (edge_avg_normal(:,Locb3_between) + face_unit_norms(:,c))*0.5;

        % edge_faces
        Edge_Faces(Locb3_between,2) = c;

    end

end
actual_n_shell_edges = shell_edge_index-1;
shell_edges = shell_edges(1:actual_n_shell_edges,:);

actual_n_hinges = hinge_index-1;
hinges = hinges(1:actual_n_hinges,:);

% edge_avg_normal = edge_avg_normal(:,1:actual_n_shell_edges);

shell_edge_faces = Edge_Faces(1:actual_n_shell_edges,:);


%% ghost edges for rod shell joint bend-twist springs
% s_nodes = zeros(n_rod_shell_joints,1);
ghost_rod_shell_joint_edges = [0,0];
for i = 1:n_rod_shell_joints
    s_node =  rod_shell_joint_edges(i,2);

    % faces for which s_node is one of its 3 nodes
    s_faces = [];
    for j = 1:n_faces
        if(find(s_node==face_nodes(j,:)))
            s_faces = [s_faces,j];
        end
    end
    s_edges = [];
    for k=1:size(s_faces,2)
        temp_edges = face_shell_edges(s_faces(k),:);
        for m=1:3
            already_added = ismember(shell_edges(temp_edges(m),:), ghost_rod_shell_joint_edges,'rows');
            if (~already_added)
            if(~ismembc(temp_edges(m),s_edges) )
                s_edges = [s_edges; temp_edges(m)];
            end
            end
        end
    end
    %     s_edges = reshape(face_shell_edges(s_faces,:),[3*size(s_faces,2),1]);
    ghost_rod_shell_joint_edges = [ghost_rod_shell_joint_edges; shell_edges(s_edges,:)];
end

rod_shell_joint_edges_total = [rod_shell_joint_edges; ghost_rod_shell_joint_edges(2:end,:)];

%% bend-twist springs
if(~isempty(rod_edges) || ~isempty(rod_shell_joint_edges))
    bend_twist_springs = [];
    bend_twist_signs  = [];
    rod_edges_modified = [rod_edges; rod_shell_joint_edges_total];
    for i=1:n_nodes
        bend_twist_center_node = i;
        % find edges that point into this node
        into = find(rod_edges_modified(:,2)==bend_twist_center_node);

        if(size(into,1)>=2)
            spring_edges_into = nchoosek(into,2);
            spring_nodes_into = [rod_edges_modified(spring_edges_into(:,1),1),  bend_twist_center_node.*ones(size(spring_edges_into,1),1),  rod_edges_modified(spring_edges_into(:,2),1)];
            bend_twist_springs = [bend_twist_springs; spring_nodes_into(:,1) spring_edges_into(:,1) spring_nodes_into(:,2) spring_edges_into(:,2) spring_nodes_into(:,3)];
            bend_twist_signs = [bend_twist_signs; 1.*ones(size(spring_edges_into,1),1), -1.*ones(size(spring_edges_into,1),1)];
        end
        % find edges that point outof this node
        outof = find(rod_edges_modified(:,1)==bend_twist_center_node);
        if(size(outof,1)>=2)
            spring_edges_outof = nchoosek(outof,2);
            spring_nodes_outof = [rod_edges_modified(spring_edges_outof(:,1),2),  bend_twist_center_node.*ones(size(spring_edges_outof,1),1),  rod_edges_modified(spring_edges_outof(:,2),2)];
            bend_twist_springs = [bend_twist_springs; spring_nodes_outof(:,1) spring_edges_outof(:,1) spring_nodes_outof(:,2) spring_edges_outof(:,2) spring_nodes_outof(:,3)];
            bend_twist_signs = [bend_twist_signs; -1.*ones(size(spring_edges_outof,1),1), 1.*ones(size(spring_edges_outof,1),1)];
        end
        % one edge in and one edge out
        if(~isempty(into) && ~isempty(outof))
            spring_edges_into_outof = combvec(into',outof')';
            spring_nodes_into_outof = [rod_edges_modified(spring_edges_into_outof(:,1),1),  bend_twist_center_node.*ones(size(spring_edges_into_outof,1),1),  rod_edges_modified(spring_edges_into_outof(:,2),2)];
            bend_twist_springs = [bend_twist_springs; spring_nodes_into_outof(:,1) spring_edges_into_outof(:,1) spring_nodes_into_outof(:,2) spring_edges_into_outof(:,2) spring_nodes_into_outof(:,3)];
            bend_twist_signs = [bend_twist_signs; 1.*ones(size(spring_edges_into_outof,1),1), 1.*ones(size(spring_edges_into_outof,1),1)];
        end
    end
else
    bend_twist_springs = [];
    bend_twist_signs  = [];
end
%% sequence edges: rod_edges, rod_shell_joint_total_edges,
% shell_edges-already_included_shell_edges
Edges = [rod_edges; rod_shell_joint_edges_total];

for i =1:size(shell_edges,1)
    if(isempty(Edges))
        Edges = [Edges; shell_edges(i,:)];
    else
        if(ismember(shell_edges(i,:),Edges,'rows'))
            continue;
        else
            Edges = [Edges; shell_edges(i,:)];
        end
    end
end
shell_edges = Edges(n_rod_edges+ n_rod_shell_joints + 1 : end, :); % rearranged
%% stretch springs
rod_stretch_springs = [rod_edges; rod_shell_joint_edges];
shell_stretch_springs = shell_edges;

%% face edges
face_edges = zeros(n_faces,3);
for i=1:n_faces
    node1_number = face_nodes(i,1);
    node2_number = face_nodes(i,2);
    node3_number = face_nodes(i,3);

    edge1_between=[node2_number, node3_number];
    edge1_between_negative=[node3_number, node2_number];
    
    edge2_between=[node3_number, node1_number];
    edge2_between_negative=[node1_number, node3_number];
    
    edge3_between=[node1_number, node2_number];
    edge3_between_negative=[node2_number, node1_number];


    if(sign_faces(i,1)==1)
        [~,edge1_ind]=(ismember(edge1_between,Edges,"rows"));
    elseif (sign_faces(i,1)==-1)
        [~,edge1_ind]=(ismember(edge1_between_negative,Edges,"rows"));
    end
    
    if(sign_faces(i,2)==1)
        [~,edge2_ind]=(ismember(edge2_between,Edges,"rows"));
    elseif (sign_faces(i,2)==-1)
        [~,edge2_ind]=(ismember(edge2_between_negative,Edges,"rows"));
    end
    
    if(sign_faces(i,3)==1)
        [~,edge3_ind]=(ismember(edge3_between,Edges,"rows"));
    elseif (sign_faces(i,3)==-1)
        [~,edge3_ind]=(ismember(edge3_between_negative,Edges,"rows"));
    end
    
    face_edges(i,:) = [edge1_ind, edge2_ind, edge3_ind];
end


