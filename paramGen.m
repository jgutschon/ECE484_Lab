clear;
OS_dec = 0.05;
T_SETTLING = 0.25;

ZETA_MAX = -log(OS_dec)/sqrt(pi^2 + log(OS_dec)^2);
THETA_MAX = rad2deg(acos(ZETA_MAX));
RE_S_MAX = -4 / T_SETTLING;

K_1 = -2.185;  % rad/Vs
TAU = 0.0178;  % s

%%%%% FIRST-ORDER C(s) %%%%%
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

C_COEFF = linsolve(A, CP_DES);

P = tf([K_1/TAU], [1 1/TAU 0]);
C = tf([C_COEFF(3) C_COEFF(4)], [C_COEFF(1) C_COEFF(2)]);

% SIMULINK PARAMETERS
T = 0.001;
D = c2d(C, T);
tfinal = 2;
sim('general_SD_model.slx', tfinal);
V_minimum = min(Uk)
V_maximum = max(Uk);
range = V_maximum - V_minimum;
% plot(Ysd)
last_2percent = find(Ysd < 0.672 | Ysd > 0.728, 1, 'last')

format long
[NUM,DEN]=tfdata(D,'v');

fprintf('float A_0 = %.15f;\n', NUM(1));
fprintf('float A_1 = %.15f;\n', NUM(2));
fprintf('float B_0 = %.15f;\n', DEN(1));
fprintf('float B_1 = %.15f;\n', DEN(2));
