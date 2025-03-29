function outside_pts = random_flap_points(p1, p2, p3, zero_natcurv)

    normal = cross(p2 - p1, p1 - p3);
    normal = normal / norm(normal); % Normalize

    % Compute edges
    edge12 = p2 - p1;
    edge23 = p3 - p2;
    edge31 = p1 - p3;

    % Generate random position along each edge (random weight between 0 and 1)
    lambda12 = rand();
    lambda23 = rand();
    lambda31 = rand();

    % Compute random points along each edge
    rand12 = p1 + lambda12 * edge12;
    rand23 = p2 + lambda23 * edge23;
    rand31 = p3 + lambda31 * edge31;

    % Compute outward in-plane perpendicular directions
    dir12 = cross(normal, edge12);
    dir23 = cross(normal, edge23);
    dir31 = cross(normal, edge31);

    dir12 = dir12 / norm(dir12); % Normalize
    dir23 = dir23 / norm(dir23);
    dir31 = dir31 / norm(dir31);

    % Compute output points by shifting randomly selected points outward
    offset = rand();
    out1 = rand12 + offset * dir12;
    out2 = rand23 + offset * dir23;
    out3 = rand31 + offset * dir31;

    if(~zero_natcurv)
        out1(3) = -1+ 2*rand();
        out2(3) = -1+ 2*rand();
        out3(3) = -1+ 2*rand();
    end

    % Store output points
    outside_pts = [out2', out3', out1'];

end