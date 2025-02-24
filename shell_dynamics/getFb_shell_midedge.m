function Fb_shell = getFb_shell_midedge(MultiRod, q, tau_0)

face_nodes = MultiRod.face_nodes_shell';
face_edges = MultiRod.face_edges';
sign_faces = MultiRod.sign_faces';
n_faces = MultiRod.n_faces;
n_DOF = MultiRod.n_DOF;
Fb_shell = zeros(n_DOF,1);

for i=1:n_faces
    Face_i_nodes = face_nodes(:,i);
    Face_i_edges = face_edges(:,i);
    
    p_is = zeros(3,3);
    xi_is = zeros(3,1);
    tau_0_is = zeros(3,3);
    
    for j=1:3
        p_is(:,j) = q(mapNodetoDOF(Face_i_nodes(j)));
        edge_dof = mapEdgetoDOFxi(MultiRod.face_shell_edges(i,j), MultiRod.n_nodes, MultiRod.n_edges_dof);
        xi_is(j) = q(edge_dof);
        tau_0_is(:,j) = tau_0(:,Face_i_edges(j));
    end
    init_ts = MultiRod.init_ts(:,:,i);
    init_cs = MultiRod.init_cs(:,i);
    init_fs = MultiRod.init_fs(:,i);
    init_xis = MultiRod.init_xis(:,i);

    s_is = sign_faces(:,i);
    [~, gradE] = Eb_gradEb_shell_midedge (MultiRod.kb, MultiRod.nu_shell, p_is(:,1), p_is(:,2), p_is(:,3), xi_is(1), xi_is(2), xi_is(3), ...
            s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3), MultiRod.faceA(i), ls, ...
            init_ts, init_cs, init_fs, init_xis);

    ind = []; % can be made better for speed (like other springs)
    for j=1:3
        ind = [ind; mapNodetoDOF(Face_i_nodes(j))];
    end
    for j=1:3
        ind = [ind; mapEdgetoDOFxi(MultiRod.face_shell_edges(i,j), MultiRod.n_nodes, MultiRod.n_edges_dof)];
    end


    Fb_shell (ind) = Fb_shell(ind) - gradE';

end

end