function plot_MultiRod(MultiRod, ctime, sim_params, optional_floor_z)
if(nargin>3)
    floor_z = optional_floor_z;
else
    floor_z = 0;
end
n_nodes=MultiRod.n_nodes;
q=MultiRod.q;
edges = MultiRod.Edges;
n_edges = MultiRod.n_edges;
n_edges_dof = MultiRod.n_edges_dof;
a1=MultiRod.a1;
a2=MultiRod.a2;
m1=MultiRod.m1;
m2=MultiRod.m2;

x1 = q(1:3:3*n_nodes);
x2 = q(2:3:3*n_nodes);
x3 = q(3:3:3*n_nodes);
L = sum(sqrt( (x1(2:end) - x1(1:end-1)).^2 + ...
 (x2(2:end) - x2(1:end-1)).^2 + ...
 (x3(2:end) - x3(1:end-1)).^2));
a1 = 0.1*L * a1;
a2 = 0.1*L * a2;
m1 = 0.1*L * m1;
m2 = 0.1*L * m2;

h1 = figure(2);
clf();
hold on

xlim(sim_params.plot_x)
ylim(sim_params.plot_y)
zlim(sim_params.plot_z)

for i=1:n_edges
    n1 = edges(i,1);
    n2 = edges(i,2);
    n1pos = q(mapNodetoDOF(n1));
    n2pos = q(mapNodetoDOF(n2));
    plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ko');

    % fixed nodes are drawn in red
    if(ismembc(n1, int64(MultiRod.fixed_nodes)))
        plot3(n1pos(1), n1pos(2), n1pos(3),'ro');
    end
    if(ismembc(n2, int64(MultiRod.fixed_nodes)))
        plot3(n2pos(1), n2pos(2), n2pos(3),'ro');
    end

    % fixed edges are drawn in red
    if(ismembc(i,MultiRod.fixed_edges))
        plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'r-');
    else
        plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'k-');

    end
end

for c=1:n_edges_dof
    n1 = edges(c,1);
    n2 = edges(c,2);
    xa = q(mapNodetoDOF(n1));
    xb = q(mapNodetoDOF(n2));
    xp = (xa+xb)/2;
    p1 = plot3( [xp(1), xp(1) + a1(c,1)], [xp(2), xp(2) + a1(c,2)], ...
    [xp(3), xp(3) + a1(c,3)], 'b--', 'LineWidth', 2);
    p2 = plot3( [xp(1), xp(1) + a2(c,1)], [xp(2), xp(2) + a2(c,2)], ...
    [xp(3), xp(3) + a2(c,3)], 'c--', 'LineWidth', 2);
    p3 = plot3( [xp(1), xp(1) + m1(c,1)], [xp(2), xp(2) + m1(c,2)], ...
    [xp(3), xp(3) + m1(c,3)], 'r-');
    p4 = plot3( [xp(1), xp(1) + m2(c,1)], [xp(2), xp(2) + m2(c,2)], ...
    [xp(3), xp(3) + m2(c,3)], 'g-');
end

if(sim_params.showFloor)
    patch([5 -5 -5 5], [5 5 -5 -5], [floor_z floor_z floor_z floor_z], [1 1 1 1])
end
hold off
title(num2str(ctime, 't=%f'));
% axis equal
view(3); % isometric view
% view(0,90); % x-y plane view
% view(90,0); % y-z plane view
% view(0,0); % x-z plane
xlabel('x');
ylabel('y');
zlabel('z');
if(n_edges_dof)
    % legend([p1,p2,p3,p4], 'a_1','a_2','m_1','m_2');
end
drawnow

end