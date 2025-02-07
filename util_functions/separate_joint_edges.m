function [rod_shell_joint_edges, rod_edges] = separate_joint_edges(triangles, edges)
if(isempty(edges))
    rod_shell_joint_edges = [];
    rod_edges = [];
    return
else
% Flatten the triangles array into a unique set of nodes
    shell_nodes = unique(triangles(:));

    % Identify edges that have at least one node in shell_nodes
    is_joint_edge = ismember(edges(:,1), shell_nodes) | ismember(edges(:,2), shell_nodes);
    
    % Extract joint edges
    joint_edges = edges(is_joint_edge, :);

    % Initialize rod_shell_joint_edges with correct ordering
    rod_shell_joint_edges = zeros(size(joint_edges));

    for i = 1:size(joint_edges, 1)
        n1 = joint_edges(i, 1);
        n2 = joint_edges(i, 2);
        
        if ismember(n1, shell_nodes)
            rod_shell_joint_edges(i, :) = [n2, n1]; % n1 is common, so it goes in second column
        else
            rod_shell_joint_edges(i, :) = [n1, n2]; % n2 is common, so it goes in second column
        end
    end

    % Update edges array by removing the identified joint edges
    rod_edges = edges(~is_joint_edge, :);
end
end
