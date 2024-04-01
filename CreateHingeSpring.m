function hinge_sp = CreateHingeSpring (hinge_sp, thetaBar, nodes_index, optional_stiffnesses)

global MultiRod

% default value for stiffnesses
if nargin > 3
  stiffness_bend = optional_stiffnesses;
else
  stiffness_bend = MultiRod.kb;
end
hinge_sp.nodes_ind = nodes_index;
hinge_sp.ind = [mapNodetoDOF(nodes_index(1)); mapNodetoDOF(nodes_index(2)); mapNodetoDOF(nodes_index(3)); mapNodetoDOF(nodes_index(4))];

hinge_sp.kb = stiffness_bend;
hinge_sp.thetaBar = thetaBar;

hinge_sp.dFb = zeros(12,1);
hinge_sp.dJb = zeros(12, 12);

end
