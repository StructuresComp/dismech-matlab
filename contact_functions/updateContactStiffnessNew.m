function contact_stiffness = updateContactStiffnessNew(fvec_exceptIMC, C, fixedDOF)
curr_max_force = 0;
curr_force = 0;
default_kc = 100;
if sum(abs(fvec_exceptIMC))==0
    contact_stiffness = default_kc;
    return;
end

for i=1:size(C,1)
    for j = 1:4
    node_DOF = mapNodetoDOF(C(i,j));
    if(sum(ismembc(node_DOF,fixedDOF))==0) % only if freeDOF
        curr_force = norm(fvec_exceptIMC(node_DOF));
    end
    curr_max_force = max(curr_force, curr_max_force);
    end
end

% contact_stiffness = 1e-2*curr_max_force;
% contact_stiffness = 1e5*curr_max_force;
contact_stiffness = 1*curr_max_force;
% contact_stiffness = 1;
