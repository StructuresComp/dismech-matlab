function Fg = getGravityForce(MultiRod,env)
if ismember("buoyancy",env.ext_force_list)
    g_adjusted = (1- env.rho/MultiRod.rho) * env.g; % adjusted to account for buoyancy
else 
    g_adjusted = env.g;
end

Fg = zeros(size(MultiRod.MassMat, 1), 1);
for cNode = 1:MultiRod.n_nodes
    ind = mapNodetoDOF(cNode);
    Fg(ind) = diag(MultiRod.MassMat(ind, ind)) .* g_adjusted;
end