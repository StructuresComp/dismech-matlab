function DT = nonUniformMesh(l, w, minMeshSize, maxMeshSize)
    % Validate input
    if minMeshSize >= maxMeshSize
        error('minMeshSize must be smaller than maxMeshSize');
    end

    % Generate boundary points with spacing at least minMeshSize
    boundary_x = [linspace(0, l, ceil(l/minMeshSize))'; ...  % Bottom edge
                  linspace(0, l, ceil(l/minMeshSize))'; ...  % Top edge
                  zeros(ceil(w/minMeshSize), 1); ...        % Left edge
                  l * ones(ceil(w/minMeshSize), 1)];        % Right edge

    boundary_y = [zeros(ceil(l/minMeshSize), 1); ...        % Bottom edge
                  w * ones(ceil(l/minMeshSize), 1); ...     % Top edge
                  linspace(0, w, ceil(w/minMeshSize))'; ... % Left edge
                  linspace(0, w, ceil(w/minMeshSize))'];    % Right edge

    % Generate sparse outer points with spacing between min and max
    n_coarse = round(l * w / (2 * maxMeshSize^2));  
    x_coarse = minMeshSize + (l - 2 * minMeshSize) * rand(n_coarse, 1);
    y_coarse = minMeshSize + (w - 2 * minMeshSize) * rand(n_coarse, 1);

    % Generate dense center points, ensuring spacing >= minMeshSize
    n_dense = 3 * n_coarse;  % More points in the dense center
    x_dense = l/2 + (l/4) * randn(n_dense, 1);
    y_dense = w/2 + (w/4) * randn(n_dense, 1);

    % Enforce minMeshSize spacing
    x_dense = min(max(round(x_dense / minMeshSize) * minMeshSize, minMeshSize), l - minMeshSize);
    y_dense = min(max(round(y_dense / minMeshSize) * minMeshSize, minMeshSize), w - minMeshSize);

    % Combine all points
    x = [x_coarse; x_dense; boundary_x];
    y = [y_coarse; y_dense; boundary_y];

    % Remove duplicate points
    points = unique([x, y], 'rows');

    % Create Delaunay triangulation
    DT = delaunayTriangulation(points(:,1), points(:,2));

end
