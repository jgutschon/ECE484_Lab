clear all;
close all;

%% Part D
% Experimental data
t_exp = xlsread('lab3partc.xlsx', 'Sheet1', 'A20002:A39992');
y_ref_exp = xlsread('lab3partc.xlsx', 'Sheet1', 'B20002:B39992');
y_exp = xlsread('lab3partc.xlsx', 'Sheet1', 'D20002:D39992');
theta_ref_exp = xlsread('lab3partc.xlsx', 'Sheet1', 'F20002:F39992');

% Specifications Output
range = 10000:19991;
y_ref_cycle = y_ref_exp(range);
y_cycle = y_exp(range);
theta_cycle = theta_ref_exp(range);

position_offset = 0.10;     % m
position_amplitude = 0.15;   % 0.10 - 0.25 m
ref_max = position_offset + position_amplitude;

theta_max = max(abs(min(theta_cycle)), abs(max(theta_cycle)))
os_perc = (max(y_cycle) - ref_max) / position_amplitude * 100
y_2settling = find(y_cycle < ref_max - position_amplitude * 0.02 | y_cycle > ref_max + position_amplitude * 0.02, 1, 'last') / 1000

figure(2);
hold on;

yyaxis left;
plot(t_exp, y_exp, '-', 'Color', '#0072BD');
plot(t_exp, y_ref_exp, '-', 'Color', '#000000');
ylim([0.05 0.3])
ylabel('Ball Position [m]');

yyaxis right;
plot(t_exp, theta_ref_exp, 'Color', '#D95319');
% ylim([-0.7 0.7])
ylabel('Reference Gear Angle [rad]');

legend('Ball Postition', 'Reference Position', 'Reference Gear Angle');
xlabel('Time [s]');

grid on;
ax = gca;
ax.YAxis(1).Color = 'black';
ax.YAxis(2).Color = 'black';

hold off;

% % Parameters Output
% format long
% [NUM,DEN]=tfdata(D_2,'v');
% fprintf('float A2_0 = %.15f;\n', NUM(1));
% fprintf('float A2_1 = %.15f;\n', NUM(2));
% fprintf('float A2_2 = %.15f;\n', NUM(3));
% fprintf('float A2_3 = %.15f;\n', NUM(4));
% fprintf('float B2_0 = %.15f;\n', DEN(1));
% fprintf('float B2_1 = %.15f;\n', DEN(2));
% fprintf('float B2_2 = %.15f;\n', DEN(3));
% fprintf('float B2_3 = %.15f;\n', DEN(4));