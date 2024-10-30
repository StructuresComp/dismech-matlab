function [Deltas] = min_distance_between_edges (edge_combos, q, scale)

assert(size(edge_combos, 2) == 4);
num_inputs = size(edge_combos, 1);
edge_combo_inputs = zeros(size(edge_combos,1),12);

for i = 1:size(edge_combos,1)
    for j = 1:size(edge_combos,2)
        edge_combo_inputs(i,3*j-2:3*j) = q(mapNodetoDOF(edge_combos(i,j)));
    end
end
    
    xis = edge_combo_inputs(:, 1:3).*scale;
    xie = edge_combo_inputs(:, 4:6).*scale;
    xjs = edge_combo_inputs(:, 7:9).*scale;
    xje = edge_combo_inputs(:, 10:12).*scale;
    
    ei = xie - xis;
    ej = xje - xjs;
    eij = xjs - xis;
    
    D1 = sum(ei.^2, 2);  % D1
    D2 = sum(ej.^2, 2);  % D2
    R = sum(ei .* ej, 2);  % R
    S1 = sum(ei .* eij, 2);  % S1
    S2 = sum(ej .* eij, 2);  % S2
    
    den = D1 .* D2 - R.^2;
    t = zeros(num_inputs, 1);
    non_parallels = (den ~= 0);
    
    t(non_parallels) = (S1(non_parallels) .* D2(non_parallels) - S2(non_parallels) .* R(non_parallels)) ./ den(non_parallels);
    t(t > 1) = 1;
    t(t < 0) = 0;
    
    u = (t .* R - S2) ./ D2;
    uf = u;
    uf(uf > 1) = 1;
    uf(uf < 0) = 0;
    
    t(uf ~= u) = (uf(uf ~= u) .* R(uf ~= u) + S1(uf ~= u)) ./ D1(uf ~= u);
    t(t > 1) = 1;
    t(t < 0) = 0;
    
    u(uf ~= u) = uf(uf ~= u);    
    Deltas = sqrt(sum((ei .* t - ej .* u - eij).^2, 2));
end
