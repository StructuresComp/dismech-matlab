close all
FONT = 'Times New Roman';
FONTSIZE = 12;
pWidth = 4; % inches
pHeight = 3;
set(groot, 'DefaultLineLineWidth', 1);

% Data from the table
YoungsModulus = [2e10, 2e9, 2e8, 2e7];
EB = [-5.7927e-05, -5.7927e-04, -5.7927e-03, -5.7927e-02];
Midedge = [-5.73277207529104e-05, -0.000574282627507370, -0.00578028148162771, -0.0435961409251513];
Hinge = [-6.72146641962215e-05, -0.000673966240792355, -0.00670313445175469, -0.0490601968314501];
Rod = [-6.04328744925992e-05, -0.000605282120736377, -0.00603288242166888, -0.0470216654246606];

% Convert absolute y-values to positive for log scale
EBAbs = abs(EB);
MidedgeAbs = abs(Midedge);
HingeAbs = abs(Hinge);
RodAbs = abs(Rod);

% Create the log-log plot
h = figure(1);
loglog(YoungsModulus, EBAbs, '-', 'DisplayName', 'EB');
hold on;
loglog(YoungsModulus, RodAbs, 's', 'DisplayName', 'Rod');
loglog(YoungsModulus, HingeAbs, '^', 'DisplayName', 'Hinge');
loglog(YoungsModulus, MidedgeAbs, 'x', 'DisplayName', 'Midedge');

grid on;
axis tight

% Labels, legend, and formatting
xlabel('Young''s Modulus, E [Pa]', 'Fontname', FONT, 'FontSize', FONTSIZE,'Interpreter', 'latex');
ylabel('Deflection, $w$ [m]', 'Fontname', FONT, 'FontSize', FONTSIZE,'Interpreter', 'latex');
AX = legend('Euler-Bernoulli', 'DER', 'Hinge Shell', 'Mid-edge Shell', 'Location', 'Best','Interpreter', 'latex');
LEG = findobj(AX, 'type', 'text');
set(LEG, 'Fontname', FONT, 'FontSize', FONTSIZE);
set(gca, 'Fontname', FONT, 'FontSize', FONTSIZE);
set(gcf, 'PaperUnits', 'inches', 'PaperPosition', [0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);

% Save the figure as PDF
saveas(h, 'compareCantileverData1.pdf');

hold off;