function [Fd, Jd] = getAerodynamicDrag_newest(q,q0,dt,env, MultiRod)
Cd = env.Cd;
rho_med = env.rho;
u = (q-q0)/dt ;
Fd = zeros(MultiRod.n_DOF,1);
Jd = zeros(MultiRod.n_DOF,MultiRod.n_DOF);

for c=1:MultiRod.n_faces 
    node1ind = MultiRod.face_nodes_shell(c,1);
    node2ind = MultiRod.face_nodes_shell(c,2);
    node3ind = MultiRod.face_nodes_shell(c,3);
    idx = [mapNodetoDOF(node1ind); mapNodetoDOF(node2ind); mapNodetoDOF(node3ind)];
    face_normal = cross( (q(mapNodetoDOF(node2ind))-q(mapNodetoDOF(node1ind))), (q(mapNodetoDOF(node3ind)) - q(mapNodetoDOF(node2ind))) ) ;
    face_A = 0.5 * norm(face_normal);
    face_unit_normal = face_normal/norm(face_normal);
    
    Fd(idx(1:3)) = Fd(idx(1:3)) + (rho_med*0.5*Cd*face_A/3).* ...
        -(dot(u(mapNodetoDOF(node1ind)), face_unit_normal).*face_unit_normal);
    Fd(idx(4:6)) = Fd(idx(4:6)) + (rho_med*0.5*Cd*face_A/3).* ...
        -(dot(u(mapNodetoDOF(node2ind)), face_unit_normal).*face_unit_normal);
    Fd(idx(7:9)) = Fd(idx(7:9)) + (rho_med*0.5*Cd*face_A/3).* ...
        -(dot(u(mapNodetoDOF(node3ind)), face_unit_normal).*face_unit_normal);

    Jd(idx(1:3),idx(1:3)) = Jd(idx(1:3),idx(1:3)) + -(rho_med*Cd*face_A/3/dt).*diag( dot(u(mapNodetoDOF(node1ind)), face_unit_normal).*face_unit_normal );
    Jd(idx(4:6),idx(4:6)) = Jd(idx(4:6),idx(4:6)) + -(rho_med*Cd*face_A/3/dt).*diag( dot(u(mapNodetoDOF(node2ind)), face_unit_normal).*face_unit_normal );
    Jd(idx(7:9),idx(7:9)) = Jd(idx(7:9),idx(7:9)) + -(rho_med*Cd*face_A/3/dt).*diag( dot(u(mapNodetoDOF(node3ind)), face_unit_normal).*face_unit_normal );    
end

%% check symmetry for stingray
% if(sum(abs(Fd(1:9)-Fd(19:27)))>10^6)
%     error("not same aerodynamic force on left and right side")
% end
% 
% if(sum(abs(Fd(10:3:16)))>10^6)
%     error("aerodynamic force centerline is in x-direction?")
% end