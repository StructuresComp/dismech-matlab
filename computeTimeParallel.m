function [a1, a2] = computeTimeParallel(MultiRod, a1_old, q0, q)

n_edges_rod = MultiRod.n_edges_rod;
tangent0 = computeTangent(MultiRod, q0); % Old tangents : ne x 3
tangent = computeTangent(MultiRod, q); % New tangents : ne x 3

a1 = zeros(n_edges_rod, 3); % First reference frame director
a2 = zeros(n_edges_rod, 3); % Second reference frame director

for c=1:n_edges_rod % Loop over edges
 
    t0 = tangent0(c,:); % Old tangent on c-th edge
    t = tangent(c,:); % New tangent on c-th edge
    
    a1_local = parallel_transport( a1_old(c,:), t0, t );
    
    % Just to be careful - to remove numerical errors
    a1_local = a1_local - dot(a1_local, t) * t; % Enforcing a1 is perpendicular to t
    a1_local = a1_local / norm(a1_local); % Enforcing a1 is a unit vector
    
    % Store
    a1(c,:) = a1_local;
    a2(c,:) = cross(t, a1_local);
end
end
