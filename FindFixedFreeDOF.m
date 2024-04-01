function [fixedDOF, freeDOF] = FindFixedFreeDOF(MultiRod)

% fixed_nodes: row vector with fixed node indices
fixed_nodes = MultiRod.fixed_nodes;

% fixed_edges: row vector with fixed edge indices
fixed_edges = MultiRod.fixed_edges;

n_nodes = MultiRod.n_nodes;
n_DOF = MultiRod.n_DOF;

fixedDOF_nodes=zeros(3,numel(fixed_nodes));
if (MultiRod.n_edges_rod)
    fixedDOF_edges=zeros(numel(fixed_edges),1);
else
    fixedDOF_edges = [];
end

for i=1:size(fixed_nodes,2)
    fixedDOF_nodes(:,i)=mapNodetoDOF(fixed_nodes(i));
end

fixedDOF_nodes_vec=reshape(fixedDOF_nodes,[numel(fixedDOF_nodes),1]);

for i=1:size(fixedDOF_edges)
    fixedDOF_edges(i)=mapEdgetoDOF(fixed_edges(i), n_nodes);
end
fixedDOF=[fixedDOF_nodes_vec; fixedDOF_edges];

dummy = ones(n_DOF, 1);
dummy(fixedDOF) = 0;
freeDOF = find( dummy == 1 );
