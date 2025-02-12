function [Ffr, Jfr] = computeFrictionForceAndJacobian(q, q0, C, Fc, Jc, ndof, find_jacob, mu_k, dt, vel_tol) % friction

num_inputs = size(C, 1);
edge_combo_input = zeros(1,12);
edge_combo_input0 = zeros(1,12);
ind = zeros(1,12);
Ffr = zeros(ndof,1);
Jfr = zeros(ndof,ndof);

for i = 1:num_inputs
    for j = 1:size(C,2)
        ind (3*j-2:3*j) = mapNodetoDOF(C(i,j));
        edge_combo_input (3*j-2:3*j) = q(ind (3*j-2:3*j));
        edge_combo_input0 (3*j-2:3*j) = q0(ind (3*j-2:3*j)); 
        fc (3*j-2:3*j) = Fc(ind (3*j-2:3*j));
        jc (3*j-2:3*j, 3*j-2:3*j) = Jc(ind (3*j-2:3*j), ind (3*j-2:3*j));
    end

    data = [edge_combo_input, edge_combo_input0];
    if (find(fc)) % only compute friction force if there is non-zero contact force
        [friction_type,ffr] = compute_friction(data, fc, mu_k, dt, vel_tol);
        Ffr(ind) = Ffr(ind) - ffr';


        if(find_jacob && friction_type~="ZeroVel")
            jfr = computeFrictionJacobian(data, fc, jc, mu_k, dt, vel_tol, friction_type);
            Jfr(ind,ind) = Jfr(ind,ind) - jfr;
        end
    end
end

end