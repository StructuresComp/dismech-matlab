classdef bendTwistSpring
    properties
        nodes_ind       % Indices of nodes involved in the bending/twisting
        edges_ind       % Indices of edges involved in the bending/twisting
        sgn             % Signs associated with the bending/twisting
        ind             % Degrees of freedom indices for the nodes and edges
        stiff_EI        % Bending stiffness
        stiff_GJ        % Torsional stiffness
        voronoiLen      % Voronoi length (undeformed)
        kappaBar        % Reference curvature
        refTwist        % Reference twist
        refTwist_init   % Initial reference twist
        dFb             % Force vector for bending
        dJb             % Jacobian matrix for bending
        dFt             % Force vector for twisting
        dJt             % Jacobian matrix for twisting
    end

    methods
        function obj = bendTwistSpring(nodes_edges_index, signs, kappaBar, refTwist, MultiRod, optional_stiffnesses_EI, optional_stiffnesses_GJ)
            % Constructor to initialize a BendTwistSpring object
            
            if nargin > 5
                obj.stiff_EI = [optional_stiffnesses_EI(1), optional_stiffnesses_EI(2)];
                obj.stiff_GJ = optional_stiffnesses_GJ;
            else
                obj.stiff_EI = [MultiRod.EI1, MultiRod.EI2];
                obj.stiff_GJ = MultiRod.GJ;
            end

            n_nodes = MultiRod.n_nodes;
            
            % Extract node and edge indices from nodes_edges_index
            obj.nodes_ind = [nodes_edges_index(1), nodes_edges_index(3), nodes_edges_index(5)];
            obj.edges_ind = [nodes_edges_index(2), nodes_edges_index(4)];
            obj.sgn = signs;
            
            % Set DOF indices for nodes and edges
            obj.ind = [mapNodetoDOF(obj.nodes_ind(1)); 
                       mapNodetoDOF(obj.nodes_ind(2)); 
                       mapNodetoDOF(obj.nodes_ind(3)); 
                       mapEdgetoDOF(obj.edges_ind(1), n_nodes); 
                       mapEdgetoDOF(obj.edges_ind(2), n_nodes)];
            
            % Set other properties
            obj.voronoiLen = 0.5*( MultiRod.refLen(obj.edges_ind(1)) + MultiRod.refLen(obj.edges_ind(1)) );
%             obj.voronoiLen = undefVoronoiLen;
            obj.kappaBar = kappaBar;
            obj.refTwist = refTwist;
            obj.refTwist_init = refTwist;

            % Initialize dFb, dJb, dFt, and dJt
            obj.dFb = zeros(11, 1);
            obj.dJb = zeros(11, 11);
            obj.dFt = zeros(11, 1);
            obj.dJt = zeros(11, 11);
        end
    end
end
