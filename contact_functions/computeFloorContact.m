function [Fgd,Jgd] = computeFloorContact(delta, kc, con_node_z, r_0, floor_z)

K1 = 15/delta;

dist = con_node_z - r_0 - floor_z;
if dist > delta
    Fgd = 0;
    Jgd = 0;
    return;
end
v = exp(-K1 * dist);
f = (-2 * v * log(v + 1)) / (K1 * (v + 1));
if(isnan(f))
assert(~isnan(f),'floor contact force is not real (NaN).');
end
Fgd = f * kc;
J = (2*v * log(v + 1) + v) / ((v + 1)^2);
Jgd = J * kc;

