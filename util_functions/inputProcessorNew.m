% Khalid Jawed, khalidjm@seas.ucla.edu

function [rod_nodes, shell_nodes, rod_edges, rod_shell_joint_edges, faceNodes] = inputProcessorNew(inputFileName)

% fid = fopen('input.txt', 'r');
fid = fopen(inputFileName, 'r');

tline = fgetl(fid);
typeSpec = '';

% Container for rod nodes
rod_nodes = [];
rod_nodeNo = 0;

% Container for shell nodes
shell_nodes = [];
shell_nodeNo = 0;

% Container for rod edges
rod_edges = [];
rod_edgeNo = 0;

% Container for rod shell joint edges
rod_shell_joint_edges = [];
rod_shell_jointNo = 0;

% Container for face_nodes
faceNodes = [];
faceNo = 0;


while ischar(tline)
%     disp(tline);
    
    if numel(tline) == 0
        
        % do nothing
        
    elseif strcmp(tline(1), '#') == true
        
        % do nothing
        
    elseif strcmp(tline(1), '*') == true
        
        typeSpec = lower( tline ); % Make everything lower case
        
    else
        
        dataLine = strsplit(tline, ',');
        
        if numel(dataLine) == 0
            % do nothing
        elseif strcmp('*rodnodes', typeSpec) == true
            
            if numel(dataLine) ~= 3
                fprintf('Warning. Invalid input for rodNodes.\n');
            else
                rod_nodeNo = rod_nodeNo + 1;
                node1 = str2double( dataLine{1} );
                node2 = str2double( dataLine{2} );
                node3 = str2double( dataLine{3} );
                rod_nodes = [rod_nodes; node1, node2, node3];
            end
            
        elseif strcmp('*rodedges', typeSpec) == true
            
            if numel(dataLine) ~= 2
                fprintf('Warning. Invalid input for rodEdges.\n');
            else
                rod_edgeNo = rod_edgeNo + 1;
                edge1 = int64( str2double( dataLine{1} ) );
                edge2 = int64( str2double( dataLine{2} ) );
                rod_edges = [rod_edges; edge1, edge2];
            end

        elseif strcmp('*facenodes', typeSpec) == true
            
            if numel(dataLine) ~= 3
                fprintf('Warning. Invalid input for faceNodes.\n');
            else
                faceNo = faceNo + 1;
                node1 = int64( str2double( dataLine{1} ) );
                node2 = int64( str2double( dataLine{2} ) );
                node3 = int64( str2double( dataLine{3} ) );
                faceNodes = [faceNodes; node1, node2, node3];
            end
            
        elseif strcmp('*shellnodes', typeSpec) == true
            
            if numel(dataLine) ~= 3
                fprintf('Warning. Invalid input for shellNodes.\n');
            else
                shell_nodeNo = shell_nodeNo + 1;
                node1 = str2double( dataLine{1} );
                node2 = str2double( dataLine{2} );
                node3 = str2double( dataLine{3} );
                shell_nodes = [shell_nodes; node1, node2, node3];
            end
                  
     
        elseif strcmp('*rodshelljointedges', typeSpec) == true
            
            if numel(dataLine) ~= 2
                fprintf('Warning. Invalid input for elStretchShell.\n');
            else
                rod_shell_jointNo = rod_shell_jointNo + 1;
                node1 = int64( str2double( dataLine{1} ) );
                node2 = int64( str2double( dataLine{2} ) );
                rod_shell_joint_edges = [rod_shell_joint_edges; node1, node2];
            end
                    
        end
    end

    tline = fgetl(fid);
end

fclose(fid);
