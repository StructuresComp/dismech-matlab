function [friction_type, ffr] = compute_friction(data, forces, mu, dt, vel_tol)
K2 = 15/vel_tol;
    % Extract position data
    x1s = data(1:3);
    x1e = data(4:6);
    x2s = data(7:9);
    x2e = data(10:12);
    x1s0 = data(13:15);
    x1e0 = data(16:18);
    x2s0 = data(19:21);
    x2e0 = data(22:24);

    % Extract contact force data
    f1s = forces(1:3);
    f1e = forces(4:6);
    f2s = forces(7:9);
    f2e = forces(10:12);

    % Compute norms
    f1s_n = norm(f1s);
    f1e_n = norm(f1e);
    f2s_n = norm(f2s);
    f2e_n = norm(f2e);

    % Normal force (fn)
    fn = norm(f1s + f1e);

    % ensure non-zero contact force

    % Compute contact point ratios
    assert(fn~=0, 'contact force is zero, still trying to compute friction');
    t1 = f1s_n / fn;
    u1 = f2s_n / fn;

    % Clamp contact point ratios to [0, 1]
    t1(t1>1) = 1;
    t1(t1<0) = 0;
    u1(u1>1) = 1;
    u1(u1<0) = 0;
    t2 = 1 - t1;
    u2 = 1 - u1;

    % Compute relative velocities
    v1s = (x1s - x1s0)/dt;
    v1e = (x1e - x1e0)/dt;
    v2s = (x2s - x2s0)/dt;
    v2e = (x2e - x2e0)/dt;

    v1 = t1 * v1s + t2 * v1e;
    v2 = u1 * v2s + u2 * v2e;
    v_rel = v1 - v2;

    % Compute tangential relative velocity
    contact_norm = (f1s + f1e) / fn;
    
    tv_rel = v_rel - (dot(v_rel,contact_norm)*contact_norm);
    tv_rel_n = norm(tv_rel);

    % Initialize output vector
    ffr = zeros(1, 12);

    % gamma: takes different value according to type of friction: 
    % zero, sliding,slipping 
    if(tv_rel_n==0)
        friction_type = "ZeroVel";
        return
    else
        if (tv_rel_n>vel_tol)
            friction_type = "Sliding";
            gamma = 1.0;
        else 
            friction_type = "Sticking";
            gamma = 2 / (1 + exp(-K2*tv_rel_n)) - 1;
        end
        
    end
    
    tv_rel_u = tv_rel / tv_rel_n;
        
    % Compute friction forces
    ffr_val = mu*gamma*tv_rel_u; 
    assert(~anynan(ffr_val),'IMC friction force is not real (NaN).');


    % Assign friction forces to output matrix
    ffr(1:3) = ffr_val*f1s_n;
    ffr(4:6) = ffr_val*f1e_n;
    ffr(7:9) = -ffr_val*f2s_n;
    ffr(10:12) = -ffr_val*f2e_n;
end
