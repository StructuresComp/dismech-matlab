close all
FONT = 'Times New Roman'; % 'cmr12'
FONTSIZE = 11;
pWidth = 9; % inches
pHeight = 3;
% colpos = [247 148 30;0 166 81;237 28 36;0 174 239; 0 0 0]/255; % colors
set(groot, 'DefaultLineLineWidth', 2);

%% 

filename1 = 'rod_contact2e6_newest.xls';
filename2 = 'rod_contact2e9_newest.xls';
qs1 = readmatrix(filename1);
qs2 = readmatrix(filename2);

time_arr = qs1(:,1);
current_pos_x_1 = qs1(:,2);
current_pos_y_1 = qs1(:,3);
current_pos_z_1 = qs1(:,4);

current_pos_x_2 = qs2(:,2);
current_pos_y_2 = qs2(:,3);
current_pos_z_2 = qs2(:,4);

h1 = figure(1);
hold on;
% plot(time_arr, current_pos_x_1, ...
%      time_arr, current_pos_y_1, ...
%      time_arr, current_pos_z_1);
% 
% 
% plot(time_arr, current_pos_x_2,'-.', ...
%      time_arr, current_pos_y_2,'-.', ...
%      time_arr, current_pos_z_2,'-.');

plot(time_arr, current_pos_z_1);


plot(time_arr, current_pos_z_2,'-.', Color=[0.9290 0.6940 0.1250]);

% title('time trajectory of the leading edge centerpoint')
% Ensure axis limits are tight
axis tight;

% Add dashed vertical lines at specific x-axis values

time_to_mark = [0.1003, 0.1135, 0.1393, 0.2016, 0.3434, 0.4015];

xline(time_to_mark(1), '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 0
xline(time_to_mark(2), '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 0.1
xline(time_to_mark(3), '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 0.22
xline(time_to_mark(4), '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 1.025
xline(time_to_mark(5), '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 1.025
xline(time_to_mark(6), '--', 'Color', 'k', 'LineWidth', 1.5); % Line at x = 1.025


% Add labels for the vertical lines
y_limits = ylim; % Get current y-axis limits
text(time_to_mark(1), y_limits(2), '\bf{a}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(time_to_mark(2), y_limits(2), '\bf{b}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(time_to_mark(3), y_limits(2), '\bf{c}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(time_to_mark(4), y_limits(2), '\bf{d}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(time_to_mark(5), y_limits(2), '\bf{e}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(time_to_mark(6), y_limits(2), '\bf{f}', 'FontSize', FONTSIZE, 'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

box on
xlabel('Simulation time, t [sec]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
ylabel('Displacement, $z$ [m]', 'Fontname',FONT,'FontSize',FONTSIZE, 'Interpreter', 'latex');
AX = legend(["2 MPa", "2 GPa"], 'Location', 'east', 'Interpreter', 'latex');
% AX = legend(["x (2 MPa)", "y (2 MPa)", "z (2 MPa)", "x (2 GPa)", " y (2 GPa)", "z (2 GPa)"], 'Location', 'east', 'Interpreter', 'latex');

LEG = findobj(AX,'type','text');
set(LEG,'Fontname',FONT,'FontSize',FONTSIZE);
set(gca,'Fontname', FONT,'FontSize',FONTSIZE);
set(gcf, 'PaperUnits','inches', 'PaperPosition',[0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);
saveas(h1, 'rod_contact_traj.pdf');
hold off;
