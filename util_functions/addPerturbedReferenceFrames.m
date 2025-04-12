function MultiRod = addPerturbedReferenceFrames(MultiRod, perturbation_strength)
% Adds slightly perturbed orthonormal reference frames to MultiRod
% Inputs:
%   MultiRod: structure containing 'tangent', 'a1', 'a2' fields
%   perturbation_strength: scalar, e.g., 0.1 for small perturbation

n_edges = MultiRod.n_edges_dof;
% MultiRod.a1_perturbed = zeros(n_edges, 3);
% MultiRod.a2_perturbed = zeros(n_edges, 3);

for c = 1:n_edges
    t = MultiRod.tangent(c, :); % fixed tangent (edge) direction
    t = t / norm(t); % ensure unit

    % Get original a1, a2 from space parallel frame
    a1 = MultiRod.a1(c, :);
    
    % Generate a small random vector orthogonal to t
    rand_vec = randn(1, 3);
    rand_vec = rand_vec - dot(rand_vec, t) * t; % project out component along t
    rand_vec = rand_vec / norm(rand_vec) * perturbation_strength;

    % Perturb a1 and re-orthonormalize using Gram-Schmidt
    a1_perturbed = a1 + rand_vec;
    a1_perturbed = a1_perturbed - dot(a1_perturbed, t) * t; % ensure orthogonal to t
    a1_perturbed = a1_perturbed / norm(a1_perturbed);

    a2_perturbed = cross(t, a1_perturbed); % ensure orthonormality
    
    MultiRod.a1(c, :) = a1_perturbed;
    MultiRod.a2(c, :) = a2_perturbed;
end
end
