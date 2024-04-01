function [MultiRod, stretch_springs, bend_twist_springs, hinge_springs] = ...
    DERfun_struct_new(MultiRod, stretch_springs, bend_twist_springs, hinge_springs)

% global MultiRod

% create local variables in function to store the struct values
n_nodes=MultiRod.n_nodes;
n_edges = MultiRod.n_edges;
n_edges_rod = MultiRod.n_edges_rod;
n_DOF=MultiRod.n_DOF;
q0=MultiRod.q0;
u=MultiRod.u;
a1=MultiRod.a1;
a2=MultiRod.a2;
freeIndex=MultiRod.freeDOF;
refTwist=MultiRod.refTwist;

global dt tol

% Guess: new DOF is same as old DOF vector
q = q0;
iter = 1; % Number of iterations
error = 10 * tol;

while error > tol

     % Compute time parallel reference frame
     [a1_iter, a2_iter] = computeTimeParallel(MultiRod, a1, q0, q);

     % Compute reference twist
     tangent = computeTangent(MultiRod, q);
     refTwist_iter = computeRefTwist_bend_twist_spring(bend_twist_springs, a1_iter, tangent, refTwist);
    
     % Compute material frame
     theta = q(3*n_nodes + 1:3*n_nodes + n_edges_rod);
     [m1, m2] = computeMaterialDirectors(a1_iter,a2_iter,theta); 

     % Force and Jacobian calculation
    [Fs, Js, stretch_springs] = getFs_struct(MultiRod, stretch_springs, q);
    [Fb, Jb, bend_twist_springs] = getFb_struct(MultiRod, bend_twist_springs, q, m1, m2);
    [Ft, Jt, bend_twist_springs] = getFt_struct(MultiRod, bend_twist_springs, q, refTwist_iter);
    [Fb_shell, Jb_shell, hinge_springs] = getFb_shell(MultiRod, hinge_springs, q);

     %% Net forces
     Forces = Fs + Fb + Ft + Fb_shell + MultiRod.W;
     JForces = Js + Jb + Jt + Jb_shell;

%      Forces = Fs + MultiRod.W;
%      JForces = Js ;

%      Forces = Fs + Ft + MultiRod.W;
%      JForces = Js + Jt;

%      Forces = Fs + Fb + MultiRod.W;
%      JForces = Js +Jb;

     % Equations of motion
     f = MultiRod.MassMat / dt * ( (q-q0)/ dt - u) - Forces;
     
     % Jacobian
     J = MultiRod.MassMat / dt^2 - JForces;
     
     f_free = f(freeIndex);
     J_free = J(freeIndex, freeIndex);
     Det_J = det(J_free); % to debug (takes very high values when the simulation starts to crash)

     % Newton's update
     dq_free = J_free \ f_free;
     q(freeIndex) = q(freeIndex) - dq_free;

     % Error
     error = sum(abs(f_free) );
     fprintf('Iter=%d, error=%f\n', iter, error);
     iter = iter + 1;
end
a1 = a1_iter;
a2 = a2_iter;
u = (q - q0) / dt;

%%
MultiRod.q=q;
MultiRod.u=u;
MultiRod.a1=a1;
MultiRod.a2=a2;
MultiRod.m1 = m1;
MultiRod.m2 = m2;

end
