function refTwist = computeRefTwist_bend_twist_spring (bend_twist_springs, a1, tangent, refTwist)

n_twist = numel (bend_twist_springs);

for c = 1:n_twist % for each bend-twist spring

     edge0 = bend_twist_springs(c).edges_ind(1);
     edge1 = bend_twist_springs(c).edges_ind(2);
     u0 = a1(edge0,:); % a1 vector of first edge
     u1 = a1(edge1,:); % a1 vector of second edge
     t0 = bend_twist_springs(c).sgn(1) * tangent(edge0,:); % tangent of first edge
     t1 = bend_twist_springs(c).sgn(2) * tangent(edge1,:); % tangent of second edge 
    
     ut = parallel_transport(u0, t0, t1);
     ut = rotateAxisAngle( ut, t1, refTwist(c));
     refTwist(c) = refTwist(c) + signedAngle(ut, u1, t1);

     
end

end
