FONT = 'Times New Roman'; % 'cmr12'
FONTSIZE = 15;
pWidth = 3; % inches
pHeight = 3;
colpos = [247 148 30;0 166 81;237 28 36;0 174 239; 0 0 0]/255; % colors
set(groot, 'DefaultLineLineWidth', 2);

%% 

filename = 'stingRay_data.xls';
qs = readmatrix(filename);

time_arr = qs(:,1);
current_pos_x = qs(:,2);
current_pos_y = qs(:,3) - qs(1,3).*ones(size(qs,1),1);
current_pos_z = qs(:,4);

h1 = figure(2);
plot(time_arr,current_pos_y);
% title('time trajectory of the leading edge centerpoint')
% Ensure axis limits are tight
axis tight;

box on
xlabel('Simulation time, t [sec]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
ylabel('Displacement, y [m]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
% AX = legend(["x", "y", "z"], 'Location', 'best', 'Interpreter', 'latex');
% LEG = findobj(AX,'type','text');
set(LEG,'Fontname',FONT,'FontSize',FONTSIZE);
set(gca,'Fontname', FONT,'FontSize',FONTSIZE);
set(gcf, 'PaperUnits','inches', 'PaperPosition',[0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);
saveas(h1, 'stingRay_y.pdf');

h2 = figure(3);
plot(time_arr,current_pos_z);

axis tight;

box on
xlabel('Simulation time, t [sec]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
ylabel('Displacement, z [m]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
% AX = legend(["x", "y", "z"], 'Location', 'best', 'Interpreter', 'latex');
% LEG = findobj(AX,'type','text');
set(LEG,'Fontname',FONT,'FontSize',FONTSIZE);
set(gca,'Fontname', FONT,'FontSize',FONTSIZE);
set(gcf, 'PaperUnits','inches', 'PaperPosition',[0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);
saveas(h2, 'stingRay_z.pdf');
