function [in_contact_edge_combos, closest_distance, num_coll] = detectCollisions(q, edge_combos, delta, contact_len, scale)
% Compute the min-distances of all possible edge combinations
minDs = min_distance_between_edges(edge_combos, q, scale);
% Compute the indices of all edge combinations within the collision limit
col_indices = find(minDs < scale*(delta + contact_len));
in_contact_edge_combos = edge_combos(col_indices,:); % set C of edge_combos "in contact"
closest_distance = min(minDs)/scale;
num_coll = size(in_contact_edge_combos);
end

