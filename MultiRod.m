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
        face_shell_edges
        face_nodes_shell
        edge_combos
        q0
        q
        u
        refLen
        voronoiRefLen
        voronoiArea
        faceA
        MassMat
        Fg
        EI1
        EI2
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
        init_ts
        init_fs
        init_cs
        init_xis
        fixed_nodes
        fixed_edges
        fixedDOF
        freeDOF
    end
    
    methods
        function obj = MultiRod(geom, material, twist_angles, Nodes, Edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, sign_faces, face_edges, face_shell_edges, sim_params, environment)
            % Declare local vars to store important parameters
            obj.r0 = geom.rod_r0;
            obj.h = geom.shell_h;
            obj.rho = material.density;
            Y_rod = material.youngs_rod;
            Y_shell = material.youngs_shell;
            nu_rod = material.poisson_rod;
            obj.nu_shell = material.poisson_shell;
            
            % Node and edge counts
            obj.n_nodes = size(Nodes,1);
            n_rod_edges = size(rod_edges,1);
            n_shell_edges = size(shell_edges,1);
            n_edges_rod_shell_joint_total = size(rod_shell_joint_total_edges,1);
            obj.n_edges = size(Edges,1);
            obj.n_edges_dof = n_rod_edges + n_edges_rod_shell_joint_total;
            obj.n_faces = size(face_nodes,1);
            obj.n_edges_rod_only = n_rod_edges;
            obj.n_edges_shell_only = n_shell_edges;

            % Store nodes and edges
            obj.Nodes = Nodes;
            obj.Edges = Edges;
            obj.face_nodes_shell = face_nodes;
            obj.face_shell_edges = face_shell_edges;
            
            % DOF vector
            obj.n_DOF = 3*obj.n_nodes + n_rod_edges + n_edges_rod_shell_joint_total;
            q0 = zeros(obj.n_DOF,1);
            q_nodes = reshape(Nodes', [numel(Nodes), 1]);
            q0(1:3*obj.n_nodes) = q_nodes;
            q0(3*obj.n_nodes + 1 : 3*obj.n_nodes + obj.n_edges_dof) = twist_angles;
            
            if sim_params.use_midedge
                obj.n_DOF = obj.n_DOF + n_shell_edges;
                xi_s = zeros(n_shell_edges, 1);
                q0 = [q0; xi_s];
            end
            
            obj.q0 = q0;
            obj.q = q0;
            
            % Reference lengths and areas
            obj.refLen = obj.calculateRefLen();
            obj.voronoiRefLen = obj.calculateVoronoiRefLen();
            obj.voronoiArea = obj.calculateVoronoiArea();
            obj.faceA = obj.calculateFaceArea();
            
            % Mass matrix
            obj.MassMat = obj.calculateMassMatrix(geom);
            % Weight
            if isfield(environment, 'g') && ~isempty(environment.g)
                obj.Fg = getGravityForce(obj,environment);
            else
                obj.Fg = [];
            end
            
            % Stiffnesses
            G_rod = Y_rod / (2 * (1 + nu_rod));

            if isfield(geom, 'Axs') && ~isempty(geom.Axs)
                obj.EA = Y_rod * geom.Axs;
            else
                obj.EA = Y_rod * pi * obj.r0^2;
            end

            if isfield(geom, 'Ixs1') && ~isempty(geom.Ixs1) && isfield(geom, 'Ixs2') && ~isempty(geom.Ixs2)
                obj.EI1 = Y_rod * geom.Ixs1;
                obj.EI2 = Y_rod * geom.Ixs2;
            else
                obj.EI1 = Y_rod * pi * obj.r0^4 / 4;
                obj.EI2 = Y_rod * pi * obj.r0^4 / 4;
            end

            if isfield(geom, 'Jxs') && ~isempty(geom.Jxs)
                obj.GJ = G_rod * geom.Jxs;
            else
                obj.GJ = G_rod * pi * obj.r0^4 / 2;
            end
       
            obj.ks = sqrt(3)/2 * Y_shell * obj.h * obj.refLen;
            obj.kb = 2/sqrt(3) * Y_shell * (obj.h^3) / 12;
            if sim_params.use_midedge
                if(obj.nu_shell==1)
                    error("poisson's ratio 1 for shell is not supported since (1-nu^2) terms appears in the denominator of the stiffness leading to inifinity")
                else
                    obj.kb = Y_shell * obj.h^3 / (24 * (1 - obj.nu_shell^2));
    %                 obj.kb = Y_shell * obj.h^3 / (24 * (1 + obj.nu_shell));  % if  nu = 1
                    obj.ks = 2*(Y_shell * obj.h/(1-obj.nu_shell^2)) * obj.refLen;
    %                 obj.ks = sqrt(3)/2 * Y_shell * obj.h * obj.refLen; % if  nu = 1
                end
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
                [obj.init_ts, obj.init_fs, obj.init_cs, obj.init_xis] = obj.initialCurvatureMidedge(); % calculate initial values for c,t,f,xi
            else
                obj.face_edges = [];
                obj.sign_faces = [];
                obj.init_ts = [];
                obj.init_cs = [];
                obj.init_fs = []; 
                obj.init_xis = [];
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

        function faceA = calculateFaceArea(obj)
            faceA = zeros(obj.n_faces, 1);
            for c = 1:size(obj.face_nodes_shell, 1)
                node1ind = obj.face_nodes_shell(c, 1);
                node2ind = obj.face_nodes_shell(c, 2);
                node3ind = obj.face_nodes_shell(c, 3);
                faceA(c) = 0.5 * norm(cross(obj.Nodes(node2ind, :) - obj.Nodes(node1ind, :), obj.Nodes(node3ind, :) - obj.Nodes(node2ind, :)));
            end
        end

        function massMat = calculateMassMatrix(obj, geom)
            m = zeros(obj.n_DOF, 1);

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
                if isfield(geom, 'Axs') && ~isempty(geom.Axs)
                    dm = obj.voronoiRefLen(cNode) * geom.Axs * obj.rho;
                else
                    dm = obj.voronoiRefLen(cNode) * pi * obj.r0^2 * obj.rho;
                end
                ind = mapNodetoDOF(cNode);
                m(ind) = m(ind) + dm * ones(3, 1);
            end

            % Rod edges
            for cEdge = 1:obj.n_edges_dof
                if isfield(geom, 'Axs') && ~isempty(geom.Axs)
                    dm = obj.refLen(cEdge) * geom.Axs * obj.rho;
                    edge_mass = dm * geom.Jxs/geom.Axs; % I = m*(J/A)
                else
                    dm = obj.refLen(cEdge) * pi * obj.r0^2 * obj.rho;
                    edge_mass = dm * obj.r0^2 / 2; % I = 1/2 m r^2
                end
                ind = mapEdgetoDOF(cEdge, obj.n_nodes);
                m(ind) = edge_mass;
            end

            massMat = diag(m);
        end


        function [init_ts, init_fs, init_cs, init_xis] = initialCurvatureMidedge(obj)

            init_ts = zeros(3,3,obj.n_faces);
            init_fs = zeros(3,obj.n_faces);
            init_cs = zeros(3,obj.n_faces);
            init_xis = zeros(3,obj.n_faces);
            
            edge_common_to = zeros(obj.n_edges,1);
            n_avg = zeros(3,obj.n_edges);
            tau_0 = zeros(3,obj.n_edges);
            e = zeros(3,obj.n_edges);
            for c = 1:obj.n_faces
                node1_number = obj.face_nodes_shell(c,1);
                node2_number = obj.face_nodes_shell(c,2);
                node3_number = obj.face_nodes_shell(c,3);
                node1_position = obj.q(3*node1_number-2:3*node1_number);
                node2_position = obj.q(3*node2_number-2:3*node2_number);
                node3_position = obj.q(3*node3_number-2:3*node3_number);

                % face normal calculation:
                face_normal = cross(([node2_position]-[node1_position]),([node3_position]-[node1_position]));
                face_unit_normal = face_normal .* 1/norm(face_normal);

                % face edge map
                edge1_idx = obj.face_edges(c,1);
                edge2_idx = obj.face_edges(c,2);
                edge3_idx = obj.face_edges(c,3);

                edge_common_to(edge1_idx) = edge_common_to(edge1_idx)+1;
                edge_common_to(edge2_idx) = edge_common_to(edge2_idx)+1;
                edge_common_to(edge3_idx) = edge_common_to(edge3_idx)+1;

                n_avg(:,edge1_idx) = n_avg(:,edge1_idx) + face_unit_normal;
                n_avg(:,edge1_idx) = n_avg(:,edge1_idx)/ norm(n_avg(:,edge1_idx));

                n_avg(:,edge2_idx) = n_avg(:,edge2_idx) + face_unit_normal;
                n_avg(:,edge2_idx) = n_avg(:,edge2_idx)/ norm(n_avg(:,edge2_idx));

                n_avg(:,edge3_idx) = n_avg(:,edge3_idx) + face_unit_normal;
                n_avg(:,edge3_idx) = n_avg(:,edge3_idx)/ norm(n_avg(:,edge3_idx));

                % ensure that edge is common to only 2 triangle faces (to avoid bugs)
                assert(edge_common_to(edge1_idx)<3, "edge is common to more than 2 faces!");
                assert(edge_common_to(edge2_idx)<3, "edge is common to more than 2 faces!");
                assert(edge_common_to(edge3_idx)<3, "edge is common to more than 2 faces!");

            end


            for i=1:obj.n_edges
                e(:,i) = obj.q(3*obj.Edges(i,2)-2:3*obj.Edges(i,2)) - obj.q(3*obj.Edges(i,1)-2:3*obj.Edges(i,1));
                tau_0(:,i) = cross(e(:,i), n_avg(:,i));
                tau_0(:,i) = tau_0(:,i)/norm(tau_0(:,i));
            end

            for i=1:obj.n_faces
                Face_i_nodes = obj.face_nodes_shell(i,:);
                Face_i_edges = obj.face_edges(i,:);

                p_is = zeros(3,3);
                xi_is = zeros(3,1);
                tau_0_is = zeros(3,3);

                for j=1:3
                    p_is(:,j) = obj.q(3*Face_i_nodes(j)-2:3*Face_i_nodes(j));
                    xi_is(j) = obj.q(3*obj.n_nodes + Face_i_edges(j));
                    tau_0_is(:,j) = tau_0(:,Face_i_edges(j));
                end
                s_is = obj.sign_faces(i,:);

                init_xis(:,i) = xi_is;

                [init_t, init_f, init_c] = obj.calculateInit_t_f_c_midedge(p_is, tau_0_is, s_is);

                init_ts(:,:,i) = init_t;
                init_fs (:,i) = init_f';
                init_cs (:,i) = init_c';

            end

        end
    end
    methods (Static)
        function [ts, fs, cs] = calculateInit_t_f_c_midedge(p_s, tau0_s, s_s)

            pi = p_s(:,1);
            pj = p_s(:,2);
            pk = p_s(:,3);

            tau_i0 = s_s(1)*tau0_s(:,1);
            tau_j0 = s_s(2)*tau0_s(:,2);
            tau_k0 = s_s(3)*tau0_s(:,3);

            % edges
            vi = pk - pj ; % 3*1 edge i vector
            vj = pi - pk ; % 3*1 edge j vector
            vk = pj - pi ; % 3*1 edge k vector

            % edge lengths
            li = norm(vi);
            lj = norm(vj);
            lk = norm(vk);

            % triangle face normal
            normal = cross(vk, vi);
            A = norm(normal)/2; % area of triangular face
            unit_norm = normal/norm(normal); % normalized triangle face normal vector

            % t_i's (tangent (perpendicular to edge, in plane of triangle) of length =
            % |vi|)
            t_i = cross(vi,unit_norm);
            t_j = cross(vj,unit_norm);
            t_k = cross(vk,unit_norm);

            % c_i's :  scalars
            c_i = 1/( A*li*dot((t_i/norm(t_i)),tau_i0) );
            c_j = 1/( A*lj*dot((t_j/norm(t_j)),tau_j0) );
            c_k = 1/( A*lk*dot((t_k/norm(t_k)),tau_k0) );

            % f_i's :  scalars
            f_i = dot(unit_norm,tau_i0);
            f_j = dot(unit_norm,tau_j0);
            f_k = dot(unit_norm,tau_k0);

            fs = [f_i, f_j, f_k]; % (1*3)

            ts = [t_i , t_j , t_k]; % t_i are columns

            cs = [c_i, c_j, c_k]; % (1*3)

        end

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
