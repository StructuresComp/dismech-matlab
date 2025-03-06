function Fb_shell = getFb_shell_midedge(MultiRod, triangle_springs, q, tau_0)

n_triangles = numel(triangle_springs);
n_DOF = MultiRod.n_DOF;

Fb_shell = zeros(n_DOF,1);

for i=1:n_triangles

    ind = triangle_springs(i).ind;

    p_is = [q(ind(1:3)), q(ind(4:6)), q(ind(7:9))];
    xi_is = [q(ind(10)); q(ind(11)); q(ind(12))];
    tau_0_is = tau_0(:,triangle_springs(i).face_edges);
   
    init_ts = MultiRod.init_ts(:,:,i);
    init_cs = MultiRod.init_cs(:,i);
    init_fs = MultiRod.init_fs(:,i);
    init_xis = MultiRod.init_xis(:,i);

    ls = [MultiRod.refLen(triangle_springs(i).face_edges)];


    s_is = triangle_springs(i).sgn;

    [~, gradE] = Eb_gradEb_shell_midedge (MultiRod.kb, MultiRod.nu_shell, p_is(:,1), p_is(:,2), p_is(:,3), xi_is(1), xi_is(2), xi_is(3), ...
            s_is(1), s_is(2), s_is(3), tau_0_is(:,1), tau_0_is(:,2), tau_0_is(:,3), MultiRod.faceA(i), ls, ...
            init_ts, init_cs, init_fs, init_xis);

    Fb_shell (ind) = Fb_shell(ind) - gradE';
end

end