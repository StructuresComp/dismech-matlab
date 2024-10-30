function bend_twist_springs = actuatekappa(bend_twist_springs, kappa_bar)

n_bend = numel(bend_twist_springs);

for i=1:n_bend
    bend_twist_springs(i).kappaBar = kappa_bar(i,:);
end

end