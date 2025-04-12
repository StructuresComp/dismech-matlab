function frame = randomOrthonormalFrame(edgeVector)
% RANDOMORTHONORMALFRAME Generates a random orthonormal reference frame
% aligned with the given edge vector in R^3.
% Input:
%   edgeVector - A 3x1 vector representing the edge in R^3.
% Output:
%   frame - A 3x3 matrix whose columns are orthonormal vectors. The first
%           column is a unit vector along the edgeVector.

% Normalize the edge vector to get the first orthonormal vector
v1 = edgeVector / norm(edgeVector);

% Generate a random vector not aligned with v1
randomVector = rand(3, 1); % random vector
if abs(dot(randomVector, v1)) > 0.9
    % If the random vector is nearly aligned, generate another random vector
    randomVector = rand(3, 1);
end

% Use the Gram-Schmidt process to generate the second orthonormal vector
v2 = randomVector - dot(randomVector, v1) * v1;
v2 = v2 / norm(v2);

% Compute the third orthonormal vector using the cross product
v3 = cross(v1, v2);

% Assemble the orthonormal frame
frame = [v2, v3];
% frame = [v1, v2, v3];
end
