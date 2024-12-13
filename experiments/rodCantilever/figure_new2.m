close all
FONT = 'Times New Roman';
FONTSIZE = 12;
pWidth = 4; % inches
pHeight = 3;

% Data from the table
YoungsModulus = [2e10, 2e9, 2e8, 2e7];
EB = [-5.7927e-05, -5.7927e-04, -5.7927e-03, -5.7927e-02];
Midedge = [-5.73277207529104e-05, -0.000574282627507370, -0.00578028148162771, -0.0435961409251513];
Hinge = [-6.72146641962215e-05, -0.000673966240792355, -0.00670313445175469, -0.0490601968314501];
Rod = [-6.04328744925992e-05, -0.000605282120736377, -0.00603288242166888, -0.0470216654246606];

% Convert absolute y-values to positive for error calculation
EBAbs = abs(EB);
MidedgeAbs = abs(Midedge);
HingeAbs = abs(Hinge);
RodAbs = abs(Rod);

Error_EB = EBAbs - EBAbs;
Error_Midedge = MidedgeAbs - EBAbs;
Error_Hinge = HingeAbs - EBAbs;
Error_Rod = RodAbs - EBAbs;
% Error_EB = EBAbs ;
% Error_Midedge = MidedgeAbs ;
% Error_Hinge = HingeAbs ;
% Error_Rod = RodAbs ;

% Create the plot with log-scaled x-axis
h = figure(1);
semilogx(YoungsModulus, Error_EB, '-', 'DisplayName', 'EB');
hold on;
semilogx(YoungsModulus, Error_Rod, '-s', 'DisplayName', 'Rod');
semilogx(YoungsModulus, Error_Hinge, '-.^', 'DisplayName', 'Hinge');
semilogx(YoungsModulus, Error_Midedge, '--x', 'DisplayName', 'Midedge');
axis tight
grid on;

% Labels, legend, and formatting
xlabel('Young''s Modulus, E [Pa]', 'Fontname', FONT, 'FontSize', FONTSIZE,'Interpreter', 'latex');
ylabel('Diffrence in Deflection, $e$ [m]', 'Fontname', FONT, 'FontSize', FONTSIZE,'Interpreter', 'latex');
AX = legend('Euler-Bernoulli', 'DER', 'Hinge Shell', 'Mid-edge Shell', 'Location', 'Best','Interpreter', 'latex');
LEG = findobj(AX, 'type', 'text');
set(LEG, 'Fontname', FONT, 'FontSize', FONTSIZE);
set(gca, 'Fontname', FONT, 'FontSize', FONTSIZE);
set(gcf, 'PaperUnits', 'inches', 'PaperPosition', [0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);

% Save the figure as PDF
saveas(h, 'compareCantileverData2.pdf');

hold off;
