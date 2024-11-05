% plot static sim configurations
close all
filename = 'pneunet_nat_curve.xlsx';
edges = MultiRod.Edges;
n_edges = size(edges,1);
qs = readmatrix(filename);
figure()
hold on
for j=1:3
    q = qs(:,j);
    for i=1:n_edges
        n1 = edges(i,1);
        n2 = edges(i,2);
        n1pos = q(mapNodetoDOF(n1));
        n2pos = q(mapNodetoDOF(n2));
        if(j==1)
            plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ko-');
        end
        if(j==2)
            plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'bo-');
        end
        if(j==3)
            plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ro-');
        end
    end
end
hold off
view(0,0); % x-z plane
xlabel('x [m]')
zlabel('z [m]')
list = ["rod 1"];
for i=2:1:n_edges
    list(end+1) = "";
end

list(end+1) = "rod 2";
for i=n_edges+2:1:2*n_edges
    list(end+1) = "";
end

list(end+1) = "rod 3";
for i=2*n_edges+2:1:3*n_edges
    list(end+1) = "";
end

legend(list)

filename = 'quarter_circle_with_gravity.xlsx';
edges = MultiRod.Edges;
n_edges = size(edges,1);
qs = readmatrix(filename);
figure()
hold on
for j=1:3
    q = qs(:,j);
    for i=1:n_edges
        n1 = edges(i,1);
        n2 = edges(i,2);
        n1pos = q(mapNodetoDOF(n1));
        n2pos = q(mapNodetoDOF(n2));
        if(j==1)
            plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ko-');
        end
        if(j==2)
            plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'bo-');
        end
        if(j==3)
            plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ro-');
        end
    end
end
hold off
view(0,0); % x-y plane
xlabel("x [m]")
zlabel('z [m]')
legend(list)