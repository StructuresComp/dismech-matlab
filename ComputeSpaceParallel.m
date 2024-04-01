function MultiRod = ComputeSpaceParallel(MultiRod) 
MultiRod.tangent = computeTangent(MultiRod, MultiRod.q0); % Tangent (along the edges)
if (~isempty(MultiRod.tangent))

    t0 = MultiRod.tangent(1,:); % Tangent on first edge
    t1 = [0 1 0]; % "arbitrary" vector
    
    a1Tmp = cross(t0, t1); % is perpendicular to both t0 and t1
    
    if abs(a1Tmp) < 1e-6 % ==0?
         t1 = [0 0 -1];
        a1Tmp = cross(t0, t1);
    end
    
    MultiRod.a1(1,:) = a1Tmp / norm(a1Tmp ); % make unit vector
    MultiRod.a2(1,:) = cross(MultiRod.tangent(1,:), MultiRod.a1(1,:)); % compute a2 perpendicular to tangent and a1
    
    % Space parallel transport to construct the reference frame
    for c=2:MultiRod.n_edges_rod
        t0 = MultiRod.tangent(c-1,:); % tanget on (c-1)th edge
        t1 = MultiRod.tangent(c,:); % tanget on cth edge
        a1_0 = MultiRod.a1(c-1, :);
        a1_l = parallel_transport( a1_0, t0, t1); % space parallel transport of a1 from t0 to t1
        MultiRod.a1(c,:) = a1_l / norm( a1_l );
        MultiRod.a2(c,:) = cross( t1, MultiRod.a1(c,:) );
    end
end