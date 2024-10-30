classdef stretchSpring
    properties
        stiff           % Stiffness of the spring
        refLen          % Reference (undeformed) length of the spring
        nodes_ind       % Indices of the nodes connected by the spring
        ind             % Degrees of Freedom indices for the nodes
        dF              % Forces generated by the spring
        dJ              % Jacobian matrix for the forces
    end

    methods
        function obj = stretchSpring(undefLen, nodes_index, MultiRod, optional_stiffness)
            % Constructor to initialize a StretchSpring object

            if nargin < 4
                stiffness = MultiRod.EA; % Use MultiRod's EA if no stiffness provided
            else
                stiffness = optional_stiffness;
            end

            obj.stiff = stiffness;
            obj.refLen = undefLen;
            obj.nodes_ind = nodes_index;
            obj.ind = [mapNodetoDOF(nodes_index(1)); mapNodetoDOF(nodes_index(2))];

            % Initialize dF and dJ
            obj.dF = zeros(6, 1);
            obj.dJ = zeros(6, 6);
        end
    end
end
