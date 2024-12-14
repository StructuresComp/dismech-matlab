close all
FONT = 'Times New Roman';
FONTSIZE = 12;
pWidth = 4; % inches
pHeight = 3;
set(groot, 'DefaultLineLineWidth', 1);

% Data from the table
YoungsModulus = [5e10, 2e10, 1e10, 5e9, 2e9, 1e9, 5e8, 2e8, 1e8, 5e7, 2e7, 1e7];
EB = [-2.3168e-05, -5.7927e-05, -1.15854e-04, -2.3168e-04, -5.7927e-04, -1.15854e-03, -2.3168e-03,-5.7927e-03, -1.15854e-02, -2.3168e-02, -5.7927e-02, -1.15854e-01];
Midedge = [-2.31474158046839e-05, -5.73277207529104e-05, -0.000115433482038402,...
    -0.000229692214897193, -0.000574282627507370, -0.00114870764793202, ...
    -0.00230575581158783, -0.00578028148162771, -0.0111705068594518, ...
    -0.0209680914084091, -0.0435961409251513, -0.0610367680461612];

Hinge = [-2.69481298263220e-05, -6.72146641962215e-05, -0.000133342437734674, ...
    -0.000269598118893999, -0.000673966240792355, -0.00134775042991742, ...
    -0.00269364804959303, -0.00670313445175469, -0.0131944425898442, ...
    -0.0249180671121231, -0.0490601968314501, -0.0661457144074156];

Rod = [-2.42058367749720e-05, -6.04328744925992e-05, -0.000120289749447809, ...
    -0.000241338392831631, -0.000605282120736377, -0.00121043266921659, ...
    -0.00241991366010611, -0.00603288242166888, -0.0119409873965692, ...
    -0.0229619918537874, -0.0470216654246606, -0.0650566454123579];


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