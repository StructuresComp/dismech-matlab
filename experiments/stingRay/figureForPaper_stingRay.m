FONT = 'Times New Roman'; % 'cmr12'
FONTSIZE = 11;
pWidth = 8; % inches
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

h1 = figure(1);
hold on;
plot(time_arr, current_pos_x, Color=[0.8500 0.3250 0.0980]);
plot(time_arr, current_pos_y, Color=[0.4660 0.6740 0.1880]);
plot(time_arr, current_pos_z, Color=[0 0.4470 0.7410]);
% title('time trajectory of the leading edge centerpoint')
% Ensure axis limits are tight
axis tight;

% Add dashed vertical lines at specific x-axis values
xline(0, '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 0
xline(0.1, '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 0.1
xline(0.22, '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 0.22
xline(1.025, '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 1.025
% Add labels for the vertical lines
y_limits = ylim; % Get current y-axis limits
text(0, y_limits(2), '\bf{a}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(0.1, y_limits(2), '\bf{b}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(0.22, y_limits(2), '\bf{c}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(1.025, y_limits(2), '\bf{d}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

box on
xlabel('Simulation time, t [sec]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
ylabel('Displacement [m]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
AX = legend(["x", "y", "z"], 'Location', 'best', 'Interpreter', 'latex');
LEG = findobj(AX,'type','text');
set(LEG,'Fontname',FONT,'FontSize',FONTSIZE);
set(gca,'Fontname', FONT,'FontSize',FONTSIZE);
set(gcf, 'PaperUnits','inches', 'PaperPosition',[0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);
saveas(h1, 'stingRay_traj.pdf');
hold off