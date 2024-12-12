close all
FONT = 'Arial';
FONTSIZE = 12;
pWidth = 4; % inches
pHeight = 3;
colpos = [247 148 30;0 166 81;237 28 36;0 174 239; 0 0 0]/255; % colors
set(groot, 'DefaultLineLineWidth', 1);
%% 

filename = 'pneunet_nat_curve.xlsx';
edges = MultiRod.Edges;
n_edges = size(edges,1);
qs = readmatrix(filename);

%%

h1 = figure(1);
hold on
for j=1:3
    q = qs(:,j);
    for i=1:n_edges
        n1 = edges(i,1);
        n2 = edges(i,2);
        n1pos = q(mapNodetoDOF(n1));
        n2pos = q(mapNodetoDOF(n2));
        if(j==1)
            plot([n1pos(1);n2pos(1)], [n1pos(3);n2pos(3)],'ko-');
        end
        if(j==2)
            plot([n1pos(1);n2pos(1)], [n1pos(3);n2pos(3)],'bx-');
        end
        if(j==3)
            plot([n1pos(1);n2pos(1)], [n1pos(3);n2pos(3)],'rs-');
        end
    end
end
hold off

list = ["$\mathbf{(a)} \bar{\kappa} = -0.0314$"];
for i=2:1:n_edges
    list(end+1) = "";
end

list(end+1) = "$\mathbf{(b)} \bar{\kappa} = -0.0629$";
for i=n_edges+2:1:2*n_edges
    list(end+1) = "";
end

list(end+1) = "$\mathbf{(c)} \bar{\kappa} = -0.0943$";
for i=2*n_edges+2:1:3*n_edges
    list(end+1) = "";
end

% h1 = figure(1);
% 
% x = 1:10;
% y = 1:10;
% 
% plot(x, y, '^', 'Color', colpos(1,:) );

box on
xlabel('$x$ [m]', 'Fontname',FONT,'FontSize',FONTSIZE,'Interpreter', 'latex');
ylabel('$z$ [m]', 'Fontname',FONT,'FontSize',FONTSIZE,'Interpreter', 'latex');
% AX = legend(list, 'Location', 'none','Interpreter', 'latex');
% LEG = findobj(AX,'type','text');
% set(LEG,'Fontname',FONT,'FontSize',FONTSIZE);
set(gca,'Fontname', FONT,'FontSize',FONTSIZE);
set(gcf, 'PaperUnits','inches', 'PaperPosition',[0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);
saveas(h1, 'pneuNet_naturalCurvature.pdf');

%% 

filename = 'quarter_circle_with_gravity.xlsx';
edges = MultiRod.Edges;
n_edges = size(edges,1);
qs = readmatrix(filename);

%%
h2 = figure(2);
hold on

for j=1:3
    q = qs(:,j);
    for i=1:n_edges
        n1 = edges(i,1);
        n2 = edges(i,2);
        n1pos = q(mapNodetoDOF(n1));
        n2pos = q(mapNodetoDOF(n2));
        if(j==1)
            plot([n1pos(1);n2pos(1)], [n1pos(3);n2pos(3)],'ko-');
        end
        if(j==2)
            plot([n1pos(1);n2pos(1)], [n1pos(3);n2pos(3)],'bx-');
        end
        if(j==3)
            plot([n1pos(1);n2pos(1)], [n1pos(3);n2pos(3)],'rs-');
        end
    end
end

list = ["$\mathbf{(a) } \bar{\kappa} = -0.0314 $"];
for i=2:1:n_edges
    list(end+1) = "";
end

list(end+1) = "$\mathbf{(b) } \bar{\kappa} = -0.0629 $";
for i=n_edges+2:1:2*n_edges
    list(end+1) = "";
end

list(end+1) = "$\mathbf{(c) } \bar{\kappa} = -0.0943 $";
for i=2*n_edges+2:1:3*n_edges
    list(end+1) = "";
end
box on
xlabel('$x$ [m]', 'Fontname',FONT,'FontSize',FONTSIZE,'Interpreter', 'latex');
ylabel('$z$ [m]', 'Fontname',FONT,'FontSize',FONTSIZE,'Interpreter', 'latex');
AX = legend(list, 'Location','best','Interpreter', 'latex');
LEG = findobj(AX,'type','text');
set(LEG,'Fontname',FONT,'FontSize',FONTSIZE);
set(gca,'Fontname', FONT,'FontSize',FONTSIZE);
set(gcf, 'PaperUnits','inches', 'PaperPosition',[0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);
saveas(h2, 'pneuNet_deformed.pdf');
