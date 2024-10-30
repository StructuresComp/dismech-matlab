function [center_hinges,sortedHinges] = hinges_on_centerline(hinge_springs, q)
n_hinge = numel(hinge_springs);
% find the hinge indices for the hinges on the centerline
center_hinges = [];
for c =1:n_hinge
    n0=hinge_springs(c).nodes_ind(1);
    n1=hinge_springs(c).nodes_ind(2);

    x0 = q(mapNodetoDOF(n0));
    x1 = q(mapNodetoDOF(n1));

    if(x0(1) == 0 && x1(1) == 0)
        center_hinges = [center_hinges; c];
    end
end

sum12 = zeros(size(center_hinges,1),1);
for i =1:size(center_hinges,1)
    % Get the indices for the first two nodes in the hinge
    n0=hinge_springs(center_hinges(i)).nodes_ind(1);
    n1=hinge_springs(center_hinges(i)).nodes_ind(2);

    x0 = q(mapNodetoDOF(n0));
    x1 = q(mapNodetoDOF(n1));

    % Sum the y components
    sum12(i) = x0(2) + x1(2);
end

% Sort the Hinges matrix based on sumAB in descending order
% [~, sortedIndices] = sort(sum12, 'descend');
[~, sortedIndices] = sort(sum12, 'ascend');
sortedHinges = center_hinges(sortedIndices, :);
