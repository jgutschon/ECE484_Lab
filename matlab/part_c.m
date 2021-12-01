clear all;
close all;

%%%% SIMULINK PARAMETERS %%%%
T = 0.001;
tfinal = 80;
period = 40;
position_offset = 0.15;     % m
position_amplitude = 0.03;   % 0.15 - 0.18 m

%%%% CONSTANTS %%%%
K_1 = -2.185;  % rad/Vs
K_2 = 0.0609;  % rad/rad
K_3 = -6.0885; % m / (s^2 * rad)
TAU = 0.0178;  % s

%%%% LAB 2 %%%%
T_SETTLING = 0.25;
RE_S_MAX = -4 / T_SETTLING;
RE_S_BALL_MAX = -0.592592592592593;

% Because P has pole at 0, that means that e_ss will reach 0
DOM_POLE_RE = RE_S_MAX * 4; % note this should be negative
NON_DOM_POLE = [1 -DOM_POLE_RE];
NON_DOM_POLES_RE = RE_S_MAX; % note this should be negative
NON_DOM_POLES_C = 9;
NON_DOM_POLES = [1 ...
    -2 * NON_DOM_POLES_RE ...
    NON_DOM_POLES_RE^2 + NON_DOM_POLES_C^2];

CP_DES = conv(NON_DOM_POLE, NON_DOM_POLES).';

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

%% LAB 3 PART C %%
P_SIMPLIFIED = THETA_TO_PHI * PHI_TO_Y;
P_MODIFIED = P_SIMPLIFIED * tf([1], [1 0]);
[P_NUM, P_DEN] = tfdata(P_MODIFIED);
P_NUM = P_NUM{1};
P_DEN = P_DEN{1};

A = [P_DEN(1)   0           0           P_NUM(1)           0    0;
    P_DEN(2)    P_DEN(1)    0           P_NUM(2)    P_NUM(1)    0;
    P_DEN(3)    P_DEN(2)    P_DEN(1)    P_NUM(3)    P_NUM(2)    P_NUM(1);
    P_DEN(4)    P_DEN(3)    P_DEN(2)    P_NUM(4)    P_NUM(3)    P_NUM(2);
    0           P_DEN(4)    P_DEN(3)    0           P_NUM(4)    P_NUM(3);
    0           0           P_DEN(4)    0           0           P_NUM(4)];

DOM_POLES_BALL_RE = 0.75 * RE_S_BALL_MAX;
DOM_POLES_BALL_C = 0;
DOM_POLES_BALL = conv([1 -DOM_POLES_BALL_RE + DOM_POLES_BALL_C * 1i], [1 -DOM_POLES_BALL_RE - DOM_POLES_BALL_C * 1i]);

NON_DOM_POLES_BALL_RE2 = 15.25 * RE_S_BALL_MAX;
NON_DOM_POLES_BALL_C2 = 0;

NON_DOM_POLES_BALL2 = conv([1 -NON_DOM_POLES_BALL_RE2 + NON_DOM_POLES_BALL_C2 * 1i], [1 -NON_DOM_POLES_BALL_RE2 - NON_DOM_POLES_BALL_C2 * 1i]);

NON_DOM_POLE = [1 -2.5 * RE_S_BALL_MAX];

CP_DES = conv(DOM_POLES_BALL, NON_DOM_POLES_BALL2);
CP_DES = conv(CP_DES, NON_DOM_POLE).';

C_2_COEFF = linsolve(A, CP_DES);
C_2 = tf([C_2_COEFF(4), C_2_COEFF(5), C_2_COEFF(6)], [C_2_COEFF(1), C_2_COEFF(2), C_2_COEFF(3) 0]);
D_2 = c2d(C_2, T, "tustin");
sim('general_SD_model.slx', tfinal);

% Parameters Output
format long
[NUM,DEN]=tfdata(D_2,'v');
fprintf('float A2_0 = %.15f;\n', NUM(1));
fprintf('float A2_1 = %.15f;\n', NUM(2));
fprintf('float A2_2 = %.15f;\n', NUM(3));
fprintf('float A2_3 = %.15f;\n', NUM(4));
fprintf('float B2_0 = %.15f;\n', DEN(1));
fprintf('float B2_1 = %.15f;\n', DEN(2));
fprintf('float B2_2 = %.15f;\n', DEN(3));
fprintf('float B2_3 = %.15f;\n', DEN(4));

%% Specifications Output
range = 40000:60000;
y_cycle = y(range);
y_rounded = round(y_cycle, 4);
y_ref_cycle = y_ref(range);
theta_cycle = theta(range);

theta_max = max(abs(min(theta_cycle)), abs(max(theta_cycle)))
ref_max = position_offset + position_amplitude;
os_perc = (max(y_rounded) - ref_max) / position_amplitude * 100
y_2settling = find(y_rounded < ref_max - position_amplitude * 0.02 | y_rounded > ref_max + position_amplitude * 0.02, 1, 'last') / 1000

%% Plot
% Simulated
range = 20000:60000;
hold on;

yyaxis left;
plot(t_sim(range), y(range), 'Color', '#0072BD');
plot(t_sim(range), y_ref(range), '-', 'Color', '#000000');
ylim([0.13 0.2])
ylabel('Ball Position [m]');

yyaxis right;
plot(t_sim(range), theta(range), 'Color', '#D95319');
ylim([-0.7 0.7])
ylabel('Reference Gear Angle [rad]');

legend('Ball Postition', 'Reference Position', 'Reference Gear Angle');
xlabel('Time [s]');

grid on;
ax = gca;
ax.YAxis(1).Color = 'black';
ax.YAxis(2).Color = 'black';

hold off;
