classdef hingeSpring
    properties
        nodes_ind       % Indices of nodes involved in the hinge spring
        ind             % Degrees of freedom indices for the nodes
        kb              % Bending stiffness
        thetaBar        % Reference angle
        dF             % Force vector for bending
        dJ             % Jacobian matrix for bending
    end

    methods
        function obj = hingeSpring(thetaBar, nodes_index, MultiRod, optional_stiffnesses)
            % Constructor to initialize a HingeSpring object
            
            if nargin > 3
                obj.kb = optional_stiffnesses;
            else
                obj.kb = MultiRod.kb;
            end
            
            % Set nodes index and DOF indices for hinge
            obj.nodes_ind = nodes_index;
            obj.ind = [mapNodetoDOF(nodes_index(1)); 
                       mapNodetoDOF(nodes_index(2)); 
                       mapNodetoDOF(nodes_index(3)); 
                       mapNodetoDOF(nodes_index(4))];
            
            % Set other properties
            obj.thetaBar = thetaBar;

            % Initialize dFb and dJb for bending
            obj.dF = zeros(12, 1);
            obj.dJ = zeros(12, 12);
        end
    end
end
