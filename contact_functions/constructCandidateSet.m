function [candidate_set, closest_dist] = constructCandidateSet(q, edge_combos, candidate_limit, scale)
% Compute the min-distances of all possible edge combinations
minDs = min_distance_between_edges(edge_combos, q, scale);
closest_dist = min(minDs)/scale;

% Compute the indices of all edge combinations within the candidate_limit
col_indices = find(minDs < candidate_limit);
candidate_set = edge_combos(col_indices,:); % set C of edge_combos "in contact"

end

