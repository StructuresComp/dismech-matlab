function [Fb_shell, Jb_shell] = getFbJb_shell_midedge(MultiRod, q, tau_0)

face_nodes = MultiRod.face_nodes_shell';
face_edges = MultiRod.face_edges';
sign_faces = MultiRod.sign_faces';
n_faces = MultiRod.n_faces;
n_DOF = MultiRod.n_DOF;

Fb_shell = zeros(n_DOF,1);
Jb_shell = zeros(n_DOF);

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

    ls = [MultiRod.refLen(Face_i_edges)];


    s_is = sign_faces(:,i);

    [~, gradE, hessE] = Eb_gradEb_hessEb_shell_midedge (MultiRod.kb, MultiRod.nu_shell, p_is(:,1), p_is(:,2), p_is(:,3), xi_is(1), xi_is(2), xi_is(3), ...
            s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3), MultiRod.faceA(i), ls, ...
            init_ts, init_cs, init_fs, init_xis);

%     [E, gradE, hessE_FDM] = Eb_gradEb_hessEb_FDM (MultiRod.kb, MultiRod.nu_shell, p_is(:,1), p_is(:,2), p_is(:,3), xi_is(1), xi_is(2), xi_is(3), ...
%             s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3), MultiRod.faceA(i), ls, ...
%             init_ts, init_cs, init_fs, init_xis);
%             init_ts, init_cs, init_fs, init_xis);
%%
% change = 1e-8;
% q_tri = [p_is(:,1); p_is(:,2); p_is(:,3); xi_is(1); xi_is(2); xi_is(3)]; % dof vector
% 
% gradE_FDM = zeros(1,12);
% hessE_FDM = zeros(12,12);
% 
% for iter = 1:12
%     q_change = q_tri;
%     q_change(iter) = q_tri(iter) + change;
%     pi_change = q_change(1:3);
%     pj_change = q_change(4:6);
%     pk_change = q_change(7:9);
%     xi_i_change = q_change(10);
%     xi_j_change = q_change(11);
%     xi_k_change = q_change(12);
% 
%     % changes in the energy
% %     [E_change, gradE_change] = Debug_Eb_gradEb_shell_midedge ...
% %     (MultiRod.kb, MultiRod.nu_shell, pi_change, pj_change, pk_change, xi_i_change, xi_j_change, xi_k_change, ...
% %     s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3), MultiRod.faceA(i), ls, ...
% %     init_ts, init_cs, init_fs, init_xis, ...
% %     t_i, t_j, t_k, c_i, c_j, c_k);
% 
%     [E_change, gradE_change] = Debug_Eb_gradEb_shell_midedge ...
%     (MultiRod.kb, MultiRod.nu_shell, pi_change, pj_change, pk_change, xi_i_change, xi_j_change, xi_k_change, ...
%     s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3), MultiRod.faceA(i), ls, ...
%     init_ts, init_cs, init_fs, init_xis);
% 
%     gradE_FDM(iter) = (E_change - E)/change;
% 
%     hessE_FDM(iter,:) = (gradE_change - gradE) .* (1/change);
% 
% end

    ind = []; % can be made better for speed (like other springs)
    for j=1:3
        ind = [ind; mapNodetoDOF(Face_i_nodes(j))];
    end
    for j=1:3
        ind = [ind; mapEdgetoDOFxi(MultiRod.face_shell_edges(i,j), MultiRod.n_nodes, MultiRod.n_edges_dof)];
    end


    Fb_shell (ind) = Fb_shell(ind) - gradE';
    Jb_shell (ind,ind) = Jb_shell(ind,ind) - hessE;
%     Jb_shell (ind,ind) = Jb_shell(ind,ind) - hessE_FDM;

% figure(15)
% subplot(2,1,1);
% plot( gradE, 'ro');
% hold on
% plot( gradE_FDM, 'b^');
% hold off
% legend('Analytical my', 'Finite Difference');
% xlabel('Index');
% ylabel('Gradient');
% subplot(2,1,2);
% plot( reshape(hessE, [144,1]), 'ro');
% hold on
% plot( reshape(hessE_FDM, [144,1]), 'b^');
% hold off
% legend('Analytical', 'Finite Difference');
% xlabel('Index');
% ylabel('Hessian');
end

end