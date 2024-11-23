function [fixedDOF, freeDOF] = FindFixedFreeDOF(fixed_nodes, fixed_edges, n_DOF, n_nodes)

% fixed_nodes: row vector with fixed node indices
% fixed_edges: row vector with fixed edge indices

fixedDOF_nodes=zeros(3,numel(fixed_nodes));
fixedDOF_edges=zeros(numel(fixed_edges),1);

for i=1:size(fixed_nodes,2)
    fixedDOF_nodes(:,i)=mapNodetoDOF(fixed_nodes(i));
end

fixedDOF_nodes_vec=reshape(fixedDOF_nodes,[numel(fixedDOF_nodes),1]);

for i=1:size(fixed_edges,2)
    fixedDOF_edges(i)=mapEdgetoDOF(fixed_edges(i), n_nodes);
end
fixedDOF=[fixedDOF_nodes_vec; fixedDOF_edges];

dummy = ones(n_DOF, 1);
dummy(fixedDOF) = 0;
freeDOF = find( dummy == 1 );
