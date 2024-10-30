function [dist, constraint_type, edge_combo_idx] = lumelskyMinDist(q, edge_combo_idx, scale)
assert(size(edge_combo_idx,2)==4)
% Type of collision: p2p, p2e or e2e using Lumelsky's algorithm
idx1_old = edge_combo_idx(1);
idx2_old = edge_combo_idx(2);
idx3_old = edge_combo_idx(3);
idx4_old = edge_combo_idx(4);

% xis = q(mapNodetoDOF(idx1_old));
% xie = q(mapNodetoDOF(idx2_old));
% xjs = q(mapNodetoDOF(idx3_old));
% xje = q(mapNodetoDOF(idx4_old));

% scale
xis = scale .* q(mapNodetoDOF(idx1_old));
xie = scale .* q(mapNodetoDOF(idx2_old));
xjs = scale .* q(mapNodetoDOF(idx3_old));
xje = scale .* q(mapNodetoDOF(idx4_old));
    
ei = xie - xis;
ej = xje - xjs;
eij = xjs - xis;

D1 = dot(ei,ei);  % D1
D2 = dot(ej,ej);  % D2
R = dot(ei,ej);  % R
S1 = dot(ei,eij);  % S1
S2 = dot(ej,eij);  % S2

den = D1*D2 - R^2;
t = 0.0;
if den ~= 0
    t = (S1 * D2 - S2 * R) / den;
end
t = fixbound(t);

u = (t * R - S2) / D2;

uf = fixbound(u);

if uf ~= u
    t = (uf * R + S1) / D1;
end
t = fixbound(t);
u = uf;

% Arrange the idx of the nodes based on the contact type
    % p2p: idx1 and idx3 are for two contact nodes
    % p2e: idx1 is for p; idx2 and idx3 are for e
    % e2e: idx1 and idx2 are for one e; idx3 and idx4 are for another e
if ((t == 0 || t == 1) && (u == 0 || u == 1)) % p2p
    if t == 0
        idx1 = idx1_old; % same
        idx2 = idx2_old; % same
    end
    if t == 1
        idx1 = idx2_old; % changed
        idx2 = idx1_old; % changed
    end
    if u == 0
        idx3 = idx3_old; % same
        idx4 = idx4_old; % same

    end
    if u == 1
        idx3 = idx4_old; % changed
        idx4 = idx3_old; % changed
    end
    constraint_type = "PointToPoint";
else
    if (t == 0 || t == 1 || u == 0 || u == 1) % p2e
        if t == 0
            idx1 = idx1_old; % same
            idx2 = idx2_old; % same
            idx3 = idx3_old; % same
            idx4 = idx4_old; % same

        elseif t == 1
            idx1 = idx2_old; % changed
            idx2 = idx1_old; % changed
            idx3 = idx3_old; % same
            idx4 = idx4_old; % same

        elseif u == 0
            idx1 = idx3_old; % changed
            idx2 = idx4_old; % changed
            idx3 = idx1_old; % changed
            idx4 = idx2_old; % changed

        elseif u == 1
            idx1 = idx4_old; % changed
            idx2 = idx3_old; % changed
            idx3 = idx1_old; % changed
            idx4 = idx2_old; % changed
        else
            error("either t or u should be 0 or 1")
        end
        constraint_type = "PointToEdge";
    
    else % e2e
        idx1 = idx1_old; % same
        idx2 = idx2_old; % same
        idx3 = idx3_old; % same
        idx4 = idx4_old; % same
        constraint_type = "EdgeToEdge";
    end
end
dist = norm(ei * t - ej * u - eij);
edge_combo_idx = [idx1, idx2, idx3, idx4];

end

function value = fixbound(value)
    value = max(0, min(1, value));
end