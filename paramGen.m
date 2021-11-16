clear all; close all;

T = 0.001;
K_1 = -2.185;  % rad/Vs
K_2 = 0.0609;  % rad/rad
K_3 = -6.0885; % m / (s^2 * rad)
TAU = 0.0178;  % s

%%%% LAB 1 %%%%
OS_dec = 0.05;
T_SETTLING = 0.25;

ZETA_MAX = -log(OS_dec)/sqrt(pi^2 + log(OS_dec)^2);
THETA_MAX = rad2deg(acos(ZETA_MAX));
RE_S_MAX = -4 / T_SETTLING;

% Because P has pole at 0, that means that e_ss will reach 0
DOM_POLE_RE = RE_S_MAX * 4; % note this should be negative
DOM_POLE = [1 -DOM_POLE_RE];
NON_DOM_POLES_RE = RE_S_MAX; % note this should be negative
NON_DOM_POLES_C = 9;
% ang = rad2deg(atan(NON_DOM_POLES_C/NON_DOM_POLES_RE))
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

% THETA_REF_TO_THETA = C_1 * P_motor / (1 + C_1 * P_motor);
THETA_TO_PHI = K_2;
PHI_TO_Y = tf([K_3], [1, 0, 0]);
% INNER_LOOP_PLANT = THETA_REF_TO_THETA * THETA_TO_PHI * PHI_TO_Y;
% P = INNER_LOOP_PLANT;

C_2 = -7 * tf([1, 0.35], [1, 2.5]);

% SIMULINK PARAMETERS
D = c2d(C_2, T);
position_offset = 0.15;     % m
position_amplitude = 0.1;   % 0.15 - 0.25 m
tfinal = 100.0;
sim('general_SD_model.slx', tfinal);
% plot(t_sim, y, t_sim, y_error, t_sim, y_ref)

format long
[NUM,DEN]=tfdata(D,'v');

fprintf('float A2_0 = %.15f;\n', NUM(1));
fprintf('float A2_1 = %.15f;\n', NUM(2));
fprintf('float B2_0 = %.15f;\n', DEN(1));
fprintf('float B2_1 = %.15f;\n', DEN(2));
