function [nodes, edges, faceNodes] = inputProcessorNew(inputFileName)

% fid = fopen('input.txt', 'r');
fid = fopen(inputFileName, 'r');

tline = fgetl(fid);
typeSpec = '';

% Container for nodes
nodes = [];
nodeNo = 0;

% Container for rod edges
edges = [];
rod_edgeNo = 0;

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
        elseif strcmp('*nodes', typeSpec) == true
            
            if numel(dataLine) ~= 3
                fprintf('Warning. Invalid input for Nodes.\n');
            else
                nodeNo = nodeNo + 1;
                node1 = str2double( dataLine{1} );
                node2 = str2double( dataLine{2} );
                node3 = str2double( dataLine{3} );
                nodes = [nodes; node1, node2, node3];
            end
            
        elseif strcmp('*edges', typeSpec) == true
            
            if numel(dataLine) ~= 2
                fprintf('Warning. Invalid input for Edges.\n');
            else
                rod_edgeNo = rod_edgeNo + 1;
                edge1 = int64( str2double( dataLine{1} ) );
                edge2 = int64( str2double( dataLine{2} ) );
                edges = [edges; edge1, edge2];
            end

        elseif strcmp('*triangles', typeSpec) == true
            
            if numel(dataLine) ~= 3
                fprintf('Warning. Invalid input for Triangles.\n');
            else
                faceNo = faceNo + 1;
                node1 = int64( str2double( dataLine{1} ) );
                node2 = int64( str2double( dataLine{2} ) );
                node3 = int64( str2double( dataLine{3} ) );
                faceNodes = [faceNodes; node1, node2, node3];
            end
            
                    
        end
    end

    tline = fgetl(fid);
end

fclose(fid);
