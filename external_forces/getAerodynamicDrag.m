function [Fd, Jd] = getAerodynamicDrag(q,q0,dt,env, MultiRod)
Cd = env.Cd;
rho_med = env.rho;
u = (q-q0)/dt ;
Fd = zeros(MultiRod.n_DOF,1);
Jd = zeros(MultiRod.n_DOF,MultiRod.n_DOF);
faceAs = MultiRod.faceA;

for c=1:MultiRod.n_faces 
    node1ind = MultiRod.face_nodes_shell(c,1);
    node2ind = MultiRod.face_nodes_shell(c,2);
    node3ind = MultiRod.face_nodes_shell(c,3);
    idx = [mapNodetoDOF(node1ind); mapNodetoDOF(node2ind); mapNodetoDOF(node3ind)];
    face_normal = cross( (q(mapNodetoDOF(node2ind))-q(mapNodetoDOF(node1ind))), (q(mapNodetoDOF(node3ind)) - q(mapNodetoDOF(node2ind))) ) ;
    face_A = faceAs(c);
    face_unit_normal = face_normal/norm(face_normal);

    q1 = q(mapNodetoDOF(node1ind));
    q2 = q(mapNodetoDOF(node2ind));
    q3 = q(mapNodetoDOF(node3ind));
    u1 = u(mapNodetoDOF(node1ind));
    u2 = u(mapNodetoDOF(node2ind));
    u3 = u(mapNodetoDOF(node3ind));

    if(dot(u1,face_unit_normal)>0)
        sign1 = -1;
    elseif(dot(u1,face_unit_normal)<0)
        sign1 = 1;
    else
        sign1 = 0;
    end

    if(dot(u2,face_unit_normal)>0)
        sign2 = -1;
    elseif(dot(u2,face_unit_normal)<0)
        sign2 = 1;
    else
        sign2 = 0;
    end

    if(dot(u3,face_unit_normal)>0)
        sign3 = -1;
    elseif(dot(u3,face_unit_normal)<0)
        sign3 = 1;
    else
        sign3 = 0;
    end

    Fd(idx(1:3)) = Fd(idx(1:3)) + sign1*(rho_med*0.5*Cd*face_A/3).* ...
        (dot(u1, face_unit_normal)^2.*face_unit_normal);
    Fd(idx(4:6)) = Fd(idx(4:6)) + sign2*(rho_med*0.5*Cd*face_A/3).* ...
        (dot(u2, face_unit_normal)^2.*face_unit_normal);
    Fd(idx(7:9)) = Fd(idx(7:9)) + sign3*(rho_med*0.5*Cd*face_A/3).* ...
        (dot(u3, face_unit_normal)^2.*face_unit_normal);

    % Jd [(1:3),:]
    Jd(idx(1:3),idx(1:3)) = Jd(idx(1:3),idx(1:3)) + sign1*(rho_med*Cd*face_A/3).*...
        (2*dot(u1,face_unit_normal)*face_unit_normal*((1/dt)*face_unit_normal'*eye(3) + ...
        u1'*gradient_of_unit_normal(face_normal,q1, q2, q3, 1)) + ...
        dot(u1,face_unit_normal)^2*gradient_of_unit_normal(face_normal,q1, q2, q3, 1));

    Jd(idx(1:3),idx(4:6)) = Jd(idx(1:3),idx(4:6)) + sign1*(rho_med*Cd*face_A/3).*...
        dot(u1,face_unit_normal)*(2*face_unit_normal*u1' + dot(u1,face_unit_normal)*eye(3)) * ...
        gradient_of_unit_normal(face_normal,q1, q2, q3, 2);

    Jd(idx(1:3),idx(7:9)) = Jd(idx(1:3),idx(7:9)) + sign1*(rho_med*Cd*face_A/3).*...
        dot(u1,face_unit_normal)*(2*face_unit_normal*u1' + dot(u1,face_unit_normal)*eye(3)) * ...
        gradient_of_unit_normal(face_normal,q1, q2, q3, 3);

    % Jd [(4:6),:]
    Jd(idx(4:6),idx(1:3)) = Jd(idx(4:6),idx(1:3)) + sign2*(rho_med*Cd*face_A/3).*...
        dot(u2,face_unit_normal)*(2*face_unit_normal*u2' + dot(u2,face_unit_normal)*eye(3)) * ...
        gradient_of_unit_normal(face_normal,q1, q2, q3, 1);

    Jd(idx(4:6),idx(4:6)) = Jd(idx(4:6),idx(4:6)) + sign2*(rho_med*Cd*face_A/3).*...
        (2*dot(u2,face_unit_normal)*face_unit_normal*((1/dt)*face_unit_normal'*eye(3) + ...
        u2'*gradient_of_unit_normal(face_normal,q1, q2, q3, 2)) + ...
        dot(u2,face_unit_normal)^2*gradient_of_unit_normal(face_normal,q1, q2, q3, 2));

    Jd(idx(4:6),idx(7:9)) = Jd(idx(4:6),idx(7:9)) + sign2*(rho_med*Cd*face_A/3).*...
        dot(u2,face_unit_normal)*(2*face_unit_normal*u2' + dot(u2,face_unit_normal)*eye(3)) * ...
        gradient_of_unit_normal(face_normal,q1, q2, q3, 3);

    % Jd [(7:9),:]
    Jd(idx(7:9),idx(1:3)) = Jd(idx(7:9),idx(1:3)) + sign3*(rho_med*Cd*face_A/3).*...
        dot(u3,face_unit_normal)*(2*face_unit_normal*u3' + dot(u3,face_unit_normal)*eye(3)) * ...
        gradient_of_unit_normal(face_normal,q1, q2, q3, 1);

    Jd(idx(7:9),idx(4:6)) = Jd(idx(7:9),idx(4:6)) + sign3*(rho_med*Cd*face_A/3).*...
        dot(u3,face_unit_normal)*(2*face_unit_normal*u3' + dot(u3,face_unit_normal)*eye(3)) * ...
        gradient_of_unit_normal(face_normal,q1, q2, q3, 2);

    Jd(idx(7:9),idx(7:9)) = Jd(idx(7:9),idx(7:9)) + sign3*(rho_med*Cd*face_A/3).*...
        (2*dot(u3,face_unit_normal)*face_unit_normal*((1/dt)*face_unit_normal'*eye(3) + ...
        u3'*gradient_of_unit_normal(face_normal,q1, q2, q3, 3)) + ...
        dot(u3,face_unit_normal)^2*gradient_of_unit_normal(face_normal,q1, q2, q3, 3));


end
