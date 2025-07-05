function stretch_springs = actuate_caterpillar(stretch_springs, ctime)
    p1 = 0.2;
    p2 = 0.4;
    p3 = 0.6;
    t_mod = mod(ctime, p3); % phase in current cycle

    if t_mod < p1
        stretch_springs(1).refLen = stretch_springs(1).refLen - 1e-3;
    elseif t_mod < p2
        stretch_springs(1).refLen = stretch_springs(1).refLen + 1e-3;
        stretch_springs(2).refLen = stretch_springs(2).refLen - 1e-3;
    else % p3
        stretch_springs(2).refLen = stretch_springs(2).refLen + 1e-3;
    end
end