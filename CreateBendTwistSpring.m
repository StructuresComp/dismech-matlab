function bend_twist_sp = CreateBendTwistSpring (bend_twist_sp,...
    nodes_edges_index, signs, undefVoronoiLen_arr, kappaBar, refTwist, optional_stiffnesses)

global MultiRod

% default value for stiffnesses
if nargin > 6
  stiffness_EI = optional_stiffnesses(1);
  stiffness_GJ = optional_stiffnesses(2);
else
  stiffness_EI = MultiRod.EI;
  stiffness_GJ = MultiRod.GJ;
end

n_nodes = MultiRod.n_nodes;

% nodes_index = nodes_edges_index(1:3);
% edges_index = nodes_edges_index(4:5);

nodes_index = [nodes_edges_index(1), nodes_edges_index(3), nodes_edges_index(5)];
edges_index = [nodes_edges_index(2), nodes_edges_index(4)];

bend_twist_sp.nodes_ind = nodes_index;
bend_twist_sp.edges_ind = edges_index;
bend_twist_sp.sgn = signs;
bend_twist_sp.ind = [mapNodetoDOF(nodes_index(1)); mapNodetoDOF(nodes_index(2)); mapNodetoDOF(nodes_index(3)); ...
    mapEdgetoDOF(edges_index(1), n_nodes); mapEdgetoDOF(edges_index(2), n_nodes)];

bend_twist_sp.stiff_EI = stiffness_EI;
bend_twist_sp.stiff_GJ = stiffness_GJ;
bend_twist_sp.voronoiLen = undefVoronoiLen_arr(nodes_index(2));
bend_twist_sp.kappaBar = kappaBar;
bend_twist_sp.refTwist = refTwist;
bend_twist_sp.refTwist_init = refTwist;

bend_twist_sp.dFb = zeros(11,1);
bend_twist_sp.dJb = zeros(11, 11);
bend_twist_sp.dFt = zeros(11,1);
bend_twist_sp.dJt = zeros(11, 11);

end
