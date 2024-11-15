function Fb_shell = getFb_shell_midedge(MultiRod, q, tau_0)

face_nodes = MultiRod.face_nodes_shell';
face_edges = MultiRod.face_edges';
sign_faces = MultiRod.sign_faces';
n_faces = MultiRod.n_faces;
n_DOF = MultiRod.n_DOF;
n_nodes = MultiRod.n_nodes;

Fb_shell = zeros(n_DOF,1);

for i=1:n_faces
    Face_i_nodes = face_nodes(:,i);
    Face_i_edges = face_edges(:,i);
    
    p_is = zeros(3,3);
    xi_is = zeros(3,1);
    tau_0_is = zeros(3,3);
    
    for j=1:3
        p_is(:,j) = q(3*Face_i_nodes(j)-2:3*Face_i_nodes(j));
        xi_is(j) = q(3*n_nodes + Face_i_edges(j));
        tau_0_is(:,j) = tau_0(:,Face_i_edges(j));
    end
    init_ts = MultiRod.init_ts(:,:,i);
    init_cs = MultiRod.init_cs(:,i);
    init_fs = MultiRod.init_fs(:,i);
    init_xis = MultiRod.init_xis(:,i);

    s_is = sign_faces(:,i);

    % [~, gradE, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~] = Energy_Grad_Hess_with2terms (MultiRod.kb, MultiRod.nu_shell, p_is(:,1), p_is(:,2), p_is(:,3), xi_is(1), xi_is(2), xi_is(3), ...
    %         s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3));
    [~, gradE, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~] = Energy_Grad_Hess_with2terms_nat_curv (MultiRod.kb, MultiRod.nu_shell, p_is(:,1), p_is(:,2), p_is(:,3), xi_is(1), xi_is(2), xi_is(3), ...
            s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3), init_ts, init_cs, init_fs, init_xis);

    ind = [];
    for j=1:3
        ind = [ind, 3*Face_i_nodes(j)-2:3*Face_i_nodes(j)];
    end
    for j=1:3
        ind = [ind, 3*n_nodes + Face_i_edges(j)];
    end


    Fb_shell (ind) = Fb_shell(ind) - gradE';

    % Energy
%             E_bending (timeStep) = E_bending (timeStep) + E ;
end

end