classdef triangleSpring
    properties
        nodes_ind       % Indices of nodes involved in the triangle spring
        edges_ind       % Indices of shell edges involved in the triangle spring
        sgn             % Signs of edges associated with the triangle spring
        ind             % Degrees of freedom indices for the nodes and edges
        kb              % Bending stiffness
        dF              % Force vector for bending
        dJ              % Jacobian matrix for bending
        face_edges      % edge indices for the triangle face (overall)
    end

    methods
        function obj = triangleSpring(face_nodes_index, face_edges_index, face_shell_edges_index, sign_edges, MultiRod, optional_stiffnesses)
            % Constructor to initialize a HingeSpring object
            
            if nargin > 5
                obj.kb = optional_stiffnesses;
            else
                obj.kb = MultiRod.kb;
            end
            
            % Set nodes index and DOF indices for hinge
            obj.nodes_ind = face_nodes_index;
            obj.edges_ind = face_shell_edges_index;
            obj.sgn = sign_edges;
            obj.face_edges = face_edges_index;
            
            % Set DOF indices for nodes and edges
            obj.ind = [mapNodetoDOF(obj.nodes_ind(1)); 
                       mapNodetoDOF(obj.nodes_ind(2)); 
                       mapNodetoDOF(obj.nodes_ind(3)); 
                       mapEdgetoDOFxi(obj.edges_ind(1), MultiRod.n_nodes, MultiRod.n_edges_dof); 
                       mapEdgetoDOFxi(obj.edges_ind(2), MultiRod.n_nodes, MultiRod.n_edges_dof);
                       mapEdgetoDOFxi(obj.edges_ind(3), MultiRod.n_nodes, MultiRod.n_edges_dof)];

            % Initialize dFb and dJb for bending
            obj.dF = zeros(12, 1);
            obj.dJ = zeros(12, 12);
        end
    end
end
