classdef MultiRod
    properties
        n_nodes
        n_edges
        n_faces
        n_edges_rod_only
        n_edges_shell_only
        n_edges_dof
        n_DOF
        Nodes
        Edges
        face_nodes_shell
        edge_combos
        q0
        q
        u
        refLen
        voronoiRefLen
        voronoiArea
        MassMat
        W
        EI
        EA
        GJ
        ks
        kb
        nu_shell
        rho
        h
        r0
        tangent
        a1
        a2
        m1
        m2
        undef_refTwist
        refTwist
        face_edges
        sign_faces
        fixed_nodes
        fixed_edges
        fixedDOF
        freeDOF
    end
    
    methods
        function obj = MultiRod(geom, material, twist_angles, Nodes, Edges, rod_nodes, shell_nodes, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, sign_faces, face_edges, sim_params)

            % Declare local vars to store important parameters
            obj.r0 = geom.rod_r0;
            obj.h = geom.shell_h;
            obj.rho = material.density;
            Y_rod = material.youngs_rod;
            Y_shell = material.youngs_shell;
            nu_rod = material.poisson_rod;
            obj.nu_shell = material.poisson_shell;
            
            % Node and edge counts
            n_rod_nodes = size(rod_nodes,1);
            n_shell_nodes = size(shell_nodes,1);
            obj.n_nodes = n_rod_nodes + n_shell_nodes;
            
            n_rod_edges = size(rod_edges,1);
            n_shell_edges = size(shell_edges,1);
            n_edges_rod_shell_joint_total = size(rod_shell_joint_total_edges,1);
            obj.n_edges = size(Edges,1);
            obj.n_edges_dof = n_rod_edges + n_edges_rod_shell_joint_total;
            obj.n_faces = size(face_nodes,1);

            % Store nodes and edges
            obj.Nodes = Nodes;
            obj.Edges = Edges;
            obj.face_nodes_shell = face_nodes;
            
            % DOF vector
            obj.n_DOF = 3*obj.n_nodes + n_rod_edges + n_edges_rod_shell_joint_total;
            q0 = zeros(obj.n_DOF,1);
            q_nodes = reshape(Nodes', [numel(Nodes), 1]);
            q0(1:3*obj.n_nodes) = q_nodes;
            q0(3*obj.n_nodes + 1 : 3*obj.n_nodes + obj.n_edges_dof) = twist_angles;
            
            if sim_params.use_midedge
                obj.n_DOF = obj.n_DOF + n_shell_edges;
                q0 = [q0; zeros(n_shell_edges, 1)];
            end
            
            obj.q0 = q0;
            obj.q = q0;
            
            % Reference lengths and areas
            obj.refLen = obj.calculateRefLen();
            obj.voronoiRefLen = obj.calculateVoronoiRefLen();
            obj.voronoiArea = obj.calculateVoronoiArea();
            
            % Mass matrix
            obj.MassMat = obj.calculateMassMatrix();
            % Weight
            obj.W = obj.calculateWeightVector(sim_params.g);
            
            % Stiffnesses
            G_rod = Y_rod / (2 * (1 + nu_rod));
            obj.EI = Y_rod * pi * obj.r0^4 / 4;
            obj.GJ = G_rod * pi * obj.r0^4 / 2;
            obj.EA = Y_rod * pi * obj.r0^2;
            
            obj.ks = sqrt(3)/2 * Y_shell * obj.h * obj.refLen.^3;
            obj.kb = 2/sqrt(3) * Y_shell * (obj.h^3) / 12;
            if sim_params.use_midedge
                obj.kb = Y_shell * obj.h^3 / (24 * (1 - obj.nu_shell^2));
            end
            
            % Other properties
            obj.edge_combos = obj.construct_possible_edge_combos([rod_edges; rod_shell_joint_edges]);
            obj.u = zeros(size(obj.q0));
            obj.a1 = zeros(obj.n_edges_dof, 3);
            obj.a2 = zeros(obj.n_edges_dof, 3);
            obj.m1 = zeros(obj.n_edges_dof, 3);
            obj.m2 = zeros(obj.n_edges_dof, 3);
            
            % Store additional shell face info if using midedge
            if sim_params.use_midedge
                obj.face_edges = face_edges;
                obj.sign_faces = sign_faces;
            end
                   
        end

        function refLen = calculateRefLen(obj)
            refLen = zeros(obj.n_edges, 1);
            for c = 1:obj.n_edges
                node1_index = obj.Edges(c, 1);
                node2_index = obj.Edges(c, 2);
                refLen(c) = norm(obj.Nodes(node2_index, :) - obj.Nodes(node1_index, :));
            end
        end

        function voronoiRefLen = calculateVoronoiRefLen(obj)
            voronoiRefLen = zeros(obj.n_nodes, 1);
            for c = 1:obj.n_edges_dof
                node1_index = obj.Edges(c, 1);
                node2_index = obj.Edges(c, 2);
                voronoiRefLen(node1_index) = voronoiRefLen(node1_index) + 0.5 * obj.refLen(c);
                voronoiRefLen(node2_index) = voronoiRefLen(node2_index) + 0.5 * obj.refLen(c);
            end
        end

        function voronoiArea = calculateVoronoiArea(obj)
            voronoiArea = zeros(obj.n_nodes, 1);
            for c = 1:size(obj.face_nodes_shell, 1)
                node1ind = obj.face_nodes_shell(c, 1);
                node2ind = obj.face_nodes_shell(c, 2);
                node3ind = obj.face_nodes_shell(c, 3);
                face_A = 0.5 * norm(cross(obj.Nodes(node2ind, :) - obj.Nodes(node1ind, :), obj.Nodes(node3ind, :) - obj.Nodes(node2ind, :)));

                voronoiArea(node1ind) = voronoiArea(node1ind) + face_A / 3;
                voronoiArea(node2ind) = voronoiArea(node2ind) + face_A / 3;
                voronoiArea(node3ind) = voronoiArea(node3ind) + face_A / 3;
            end
        end

        function massMat = calculateMassMatrix(obj)
            m = zeros(numel(obj.Nodes), 1);

            % Shell faces
            for i = 1:obj.n_faces
                node1ind = obj.face_nodes_shell(i, 1);
                node2ind = obj.face_nodes_shell(i, 2);
                node3ind = obj.face_nodes_shell(i, 3);
                face_A = 0.5 * norm(cross((obj.Nodes(node2ind, :) - obj.Nodes(node1ind, :)), (obj.Nodes(node3ind, :) - obj.Nodes(node2ind, :))));
                Mface = obj.rho * face_A * obj.h;

                m(mapNodetoDOF(node1ind)) = m(mapNodetoDOF(node1ind)) + Mface / 3 * ones(3, 1);
                m(mapNodetoDOF(node2ind)) = m(mapNodetoDOF(node2ind)) + Mface / 3 * ones(3, 1);
                m(mapNodetoDOF(node3ind)) = m(mapNodetoDOF(node3ind)) + Mface / 3 * ones(3, 1);
            end

            % Rod nodes
            for cNode = 1:obj.n_nodes
                dm = obj.voronoiRefLen(cNode) * pi * obj.r0^2 * obj.rho;
                ind = mapNodetoDOF(cNode);
                m(ind) = m(ind) + dm * ones(3, 1);
            end

            % Rod edges
            for cEdge = 1:obj.n_edges_dof
                dm = obj.refLen(cEdge) * pi * obj.r0^2 * obj.rho;
                ind = mapEdgetoDOF(cEdge, obj.n_nodes);
                m(ind) = dm / 2 * obj.r0^2;  % I = 1/2 m r^2
            end

            massMat = diag(m);
        end

        function W = calculateWeightVector(obj,g)
            W = zeros(size(obj.MassMat, 1), 1);
            for cNode = 1:obj.n_nodes
                ind = mapNodetoDOF(cNode);
                W(ind) = diag(obj.MassMat(ind, ind)) .* g;
            end
        end
    end
    methods (Static)
        function [edge_combos, edge_combos_idx] = construct_possible_edge_combos(edges)
            % Construct list of all possible edge combinations without duplicates (excluding adjacent edges)
            % Inputs:
            % edges:- n_edges*2 array of edge node indices
            % Outputs:
            % edge_combos:- no. of possible edge_combos for collision*4 (node indices xi, xi+1, xj, xj+1)
            % edge_combos_idx:- no. of possible edge_combos for collision*2 (edge indices ei, ej)
            % ___________________________________________________________________________________________

            no_of_edges = size(edges, 1);

            edge_combos_idx = [0, 0]; % jugaad for using ismember
            for i=1:no_of_edges
                for j=1:no_of_edges
                    temp_combo = [i , j];
                    % check if edge is itself or adjacent
                    if(edges(i,1) == edges(j,1) || edges(i,1) == edges(j,2) || edges(i,2) == edges(j,1) || edges(i,2) == edges(j,2) )
                        % not valid combination
                    elseif (ismember(temp_combo, edge_combos_idx, "rows") || ismember([j,i], edge_combos_idx,"rows"))
                        % already counted combination
                    else
                        edge_combos_idx = [edge_combos_idx; temp_combo];
                    end
                end
            end
            edge_combos_idx = edge_combos_idx(2:end,:); % remove the jugaad for using ismember
            edge_combos = zeros(size(edge_combos_idx,1),4);

            for k = 1:size(edge_combos_idx,1)
                edge_combos(k,:) = [edges(edge_combos_idx(k,1),:), edges(edge_combos_idx(k,2),:)];
            end

        end
    end
end
