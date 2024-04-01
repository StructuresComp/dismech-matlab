% Khalid Jawed, khalidjm@seas.ucla.edu

function [nodes, edges, faceNodes, elStretchRod, elBendRod, elBendSign, elStretchShell, elBendShell] = inputProcessor(inputFileName)

% fid = fopen('input.txt', 'r');
fid = fopen(inputFileName, 'r');

tline = fgetl(fid);
typeSpec = '';

% Container for nodes
nodes = [];
nodeNo = 0;

% Container for edges
edges = [];
edgeNo = 0;

% Container for face_nodes
faceNodes = [];
faceNo = 0;

% Container for elStretchRod
elStretchRod = [];
elStretchRodNo = 0;

% Container for elBendRod
elBendRod = [];
elBendRodNo = 0;

% Container for elBendSign
elBendSign = [];
elBendSignNo = 0;

% Container for elStretchShell
elStretchShell = [];
elStretchShellNo = 0;

% Container for elBendShell
elBendShell = [];
elBendShellNo = 0;

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
                fprintf('Warning. Invalid input for nodes.\n');
            else
                nodeNo = nodeNo + 1;
                node1 = str2double( dataLine{1} );
                node2 = str2double( dataLine{2} );
                node3 = str2double( dataLine{3} );
                nodes = [nodes; node1, node2, node3];
            end
            
        elseif strcmp('*edges', typeSpec) == true
            
            if numel(dataLine) ~= 2
                fprintf('Warning. Invalid input for edges.\n');
            else
                edgeNo = edgeNo + 1;
                edge1 = int64( str2double( dataLine{1} ) );
                edge2 = int64( str2double( dataLine{2} ) );
                edges = [edges; edge1, edge2];
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
            
        elseif strcmp('*elstretchrod', typeSpec) == true
            
            if numel(dataLine) ~= 2
                fprintf('Warning. Invalid input for elStretchRod.\n');
            else
                elStretchRodNo = elStretchRodNo + 1;
                node1 = int64( str2double( dataLine{1} ) );
                node2 = int64( str2double( dataLine{2} ) );
                elStretchRod = [elStretchRod; node1, node2];
            end
            
        elseif strcmp('*elbendrod', typeSpec) == true
            
            if numel(dataLine) ~= 5
                fprintf('Warning. Invalid input for elBendRod.\n');
            else
                elBendRodNo = elBendRodNo + 1;
                node1 = int64( str2double( dataLine{1} ) );
                edge1 = int64( str2double( dataLine{2} ) );
                node2 = int64( str2double( dataLine{3} ) );
                edge2 = int64( str2double( dataLine{4} ) );
                node3 = int64( str2double( dataLine{5} ) );
                elBendRod = [elBendRod; node1, edge1, node2, edge2, node3];
            end

         elseif strcmp('*elbendsign', typeSpec) == true
            
            if numel(dataLine) ~= 2
                fprintf('Warning. Invalid input for elBendSign.\n');
            else
                elBendSignNo = elBendSignNo + 1;
                sign1 = ( str2double( dataLine{1} ) ) ;
                sign2 = ( str2double( dataLine{2} ) ) ;
                elBendSign = [elBendSign; sign1, sign2];
            end
        elseif strcmp('*elstretchshell', typeSpec) == true
            
            if numel(dataLine) ~= 2
                fprintf('Warning. Invalid input for elStretchShell.\n');
            else
                elStretchShellNo = elStretchShellNo + 1;
                node1 = int64( str2double( dataLine{1} ) );
                node2 = int64( str2double( dataLine{2} ) );
                elStretchShell = [elStretchShell; node1, node2];
            end
            
        elseif strcmp('*elbendshell', typeSpec) == true            
            
            if numel(dataLine) ~= 4
                fprintf('Warning. Invalid input for elBendShell.\n');
            else
                elBendShellNo = elBendShellNo + 1;
                node1 = int64( str2double( dataLine{1} ) );
                node2 = int64( str2double( dataLine{2} ) );
                node3 = int64( str2double( dataLine{3} ) );
                node4 = int64( str2double( dataLine{4} ) );
                elBendShell = [elBendShell; node1, node2, node3, node4];
            end
            
        end
    end

    tline = fgetl(fid);
end

fclose(fid);
