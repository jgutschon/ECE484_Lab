close all;

%%%% SIMULINK PARAMETERS %%%%
T = 0.001;
position_offset = 0.15;     % m
position_amplitude = 0.03;   % 0.15 - 0.25 m
tfinal = 38.0;

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

% LAB 3 PART B %%
format long
[NUM,DEN]=tfdata(D_2,'v');
fprintf('float A2_0 = %.15f;\n', NUM(1));
fprintf('float A2_1 = %.15f;\n', NUM(2));
fprintf('float B2_0 = %.15f;\n', DEN(1));
fprintf('float B2_1 = %.15f;\n', DEN(2));
