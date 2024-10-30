function [m1, m2] = computeMaterialDirectors(a1, a2, theta)

n_edges_dof=numel(theta);

m1 = zeros(n_edges_dof, 3); % Material frame director 1 for all the edges
m2 = zeros(n_edges_dof, 3); % Material frame director 2 for all the edges

for c=1:n_edges_dof
     m1(c,:) = cos(theta(c)) * a1(c,:) + sin(theta(c)) * a2(c,:);
     m2(c,:) = - sin(theta(c)) * a1(c,:) + cos(theta(c)) * a2(c,:);
end

end