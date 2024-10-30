% plot static sim configurations
filename = 'quarter_circle.xlsx';
edges = MultiRod.Edges;
n_edges = size(edges,1);
qs = readmatrix(filename);
figure()
for j=1:3
    q = qs(:,j);
    for i=1:n_edges
        n1 = edges(i,1);
        n2 = edges(i,2);
        n1pos = q(mapNodetoDOF(n1));
        n2pos = q(mapNodetoDOF(n2));
        plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ko-');
        hold on
    end
end
view(0,0); % x-s plane

filename = 'quarter_circle_with_gravity.xlsx';
edges = MultiRod.Edges;
n_edges = size(edges,1);
qs = readmatrix(filename);
figure()
for j=1:3
    q = qs(:,j);
    for i=1:n_edges
        n1 = edges(i,1);
        n2 = edges(i,2);
        n1pos = q(mapNodetoDOF(n1));
        n2pos = q(mapNodetoDOF(n2));
        plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'ko-');
        hold on
    end
end
view(0,0); % x-s plane