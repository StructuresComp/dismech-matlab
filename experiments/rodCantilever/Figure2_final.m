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

% 
% YoungsModulus = [ 1e10, 1e9, 1e8, 1e7];
% EB = [-1.15854e-04, -1.15854e-03, -1.15854e-02, -1.15854e-01];
% Midedge = [ -0.000115433482038402, -0.00114870764793202, -0.0111705068594518, -0.0610367680461612];
% Hinge = [ -0.000133342437734674, -0.00134775042991742,  -0.0131944425898442, -0.0661457144074156];
% Rod = [ -0.000120289749447809, -0.00121043266921659, -0.0119409873965692, -0.0650566454123579];

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
% semilogx(YoungsModulus, Error_EB, '-', 'DisplayName', 'EB');

semilogx(YoungsModulus, Error_Rod, '-s', 'DisplayName', 'Rod', Color=[0.8500 0.3250 0.0980]);
hold on;
semilogx(YoungsModulus, Error_Hinge, '-.^', 'DisplayName', 'Hinge', Color=[0.9290 0.6940 0.1250]);
semilogx(YoungsModulus, Error_Midedge, '--x', 'DisplayName', 'Midedge', Color=[0.4940 0.1840 0.5560]);
% axis tight
xlim('tight')
ylim([-0.06, 0.06])
grid on;

% Labels, legend, and formatting
xlabel('Young''s Modulus, E [Pa]', 'Fontname', FONT, 'FontSize', FONTSIZE,'Interpreter', 'latex');
ylabel('Difference in Deflection, $e$ [m]', 'Fontname', FONT, 'FontSize', FONTSIZE,'Interpreter', 'latex');
AX = legend( 'DER', 'Hinge Shell', 'Mid-edge Shell', 'Location', 'Best','Interpreter', 'latex');
LEG = findobj(AX, 'type', 'text');
set(LEG, 'Fontname', FONT, 'FontSize', FONTSIZE);
set(gca, 'Fontname', FONT, 'FontSize', FONTSIZE);
set(gcf, 'PaperUnits', 'inches', 'PaperPosition', [0 0 pWidth pHeight], ...
    'PaperSize', [pWidth pHeight]);

% Save the figure as PDF
saveas(h, 'compareCantileverData2.pdf');

hold off;
