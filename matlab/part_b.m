clear all;
close all;

%%%% SIMULINK PARAMETERS %%%%
T = 0.001;
tfinal = 80;
period = 20;
position_offset = 0.15;     % m
position_amplitude = 0.1;   % 0.15 - 0.25 m

%%%% CONSTANTS %%%%
K_1 = -2.185;  % rad/Vs
K_2 = 0.0609;  % rad/rad
K_3 = -6.0885; % m / (s^2 * rad)
TAU = 0.0178;  % s

%%%% LAB 2 %%%%
OS_dec = 0.05;
T_SETTLING = 0.25;
RE_S_MAX = -4 / T_SETTLING;

% Because P has pole at 0, that means that e_ss will reach 0
DOM_POLE_RE = RE_S_MAX * 4; % note this should be negative
DOM_POLE = [1 -DOM_POLE_RE];
NON_DOM_POLES_RE = RE_S_MAX; % note this should be negative
NON_DOM_POLES_C = 9;
NON_DOM_POLES = [1 ...
    -2 * NON_DOM_POLES_RE ...
    NON_DOM_POLES_RE^2 + NON_DOM_POLES_C^2];

CP_DES = conv(DOM_POLE, NON_DOM_POLES).';

A = [1 0 0 0;
    1/TAU 1 0 0;
    0 1/TAU K_1/TAU 0;
    0 0 0 K_1/TAU];

C_1_COEFF = linsolve(A, CP_DES);

P_motor = tf([K_1], [TAU 1 0]);
C_1 = tf([C_1_COEFF(3) C_1_COEFF(4)], [C_1_COEFF(1) C_1_COEFF(2)]);
D_1 = c2d(C_1, T);

THETA_REF_TO_THETA =tf(zpk(C_1*P_motor/(C_1*P_motor+1)));
THETA_TO_PHI = K_2;
PHI_TO_Y = tf([K_3], [1, 0, 0]);
INNER_LOOP_PLANT = THETA_REF_TO_THETA * THETA_TO_PHI * PHI_TO_Y;
P = INNER_LOOP_PLANT;

%% LAB 3 PART A %%

C_2 = -7 * tf([1, 0.35], [1, 2.5]);

% SIMULINK PARAMETERS
D_2 = c2d(C_2, T);
sim('general_SD_model.slx', tfinal);

%% LAB 3 PART B %%
format long
[NUM,DEN]=tfdata(D_2,'v');
fprintf('float A2_0 = %.15f;\n', NUM(1));
fprintf('float A2_1 = %.15f;\n', NUM(2));
fprintf('float B2_0 = %.15f;\n', DEN(1));
fprintf('float B2_1 = %.15f;\n', DEN(2));

%% Specifications Output
% Experimental Data
t_exp = xlsread('lab3partb.xlsx', 'Sheet1', 'A20002:A39992');
y_ref_exp = xlsread('lab3partb.xlsx', 'Sheet1', 'B20002:B39992');
y_exp = xlsread('lab3partb.xlsx', 'Sheet1', 'D20002:D39992');
theta_ref_exp = xlsread('lab3partb.xlsx', 'Sheet1', 'F20002:F39992');

% Simulated
range = 40000:50000;
y_cycle = y(range);
y_ref_cycle = y_ref(range);
theta_cycle = theta(range);

theta_max = max(abs(min(theta_cycle)), abs(max(theta_cycle)))
ref_max = position_offset + position_amplitude;
os_perc = (max(y_cycle) - ref_max) / position_amplitude * 100
y_2settling = find(y_cycle < ref_max - position_amplitude * 0.02 | y_cycle > ref_max + position_amplitude * 0.02, 1, 'last') / 1000

% Experimental
range = 10000:19990;
y_cycle = y_exp(range);
y_ref_cycle = y_ref_exp(range);
theta_cycle = theta_ref_exp(range);

theta_max = max(abs(min(theta_cycle)), abs(max(theta_cycle)))
ref_max = position_offset + position_amplitude;
os_perc = (max(y_cycle) - ref_max) / position_amplitude * 100
y_2settling = find(y_cycle < ref_max - position_amplitude * 0.02 | y_cycle > ref_max + position_amplitude * 0.02, 1, 'last') / 1000

%% Plot
range = 30000:50000;
figure(1);
hold on;

% shift time to match experimental
t_sim_shift = t_sim - 10;

yyaxis left;
plot(t_sim_shift(range), y(range), 'Color', '#0072BD');
plot(t_exp, y_exp, '-', 'Color', '#77AC30');
plot(t_sim_shift(range), y_ref(range), '-', 'Color', '#000000');
% ylim([0.13 0.2])
ylabel('Ball Position [m]');

yyaxis right;
plot(t_sim_shift(range), theta(range), '-', 'Color', '#FF0000');
plot(t_exp, theta_ref_exp, '-', 'Color', '#EDB120');
% ylim([-0.7 0.7])
ylabel('Reference Gear Angle [rad]');

legend( ...
    'Ball Position (sim)', ...
    'Ball Position (exp)', ...
    'Reference Ball Position', ...
    'Reference Gear Angle (sim)', ...
    'Reference Gear Angle (exp)', ...
    'Location', 'southeast' ...
);

% xlim([30 50]);
xlabel('Time [s]');

grid on;
ax = gca;
ax.YAxis(1).Color = 'black';
ax.YAxis(2).Color = 'black';

hold off;
