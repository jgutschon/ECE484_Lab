close all;

%%%% SIMULINK PARAMETERS %%%%
T = 0.001;
position_offset = 0.10;     % m
position_amplitude = 0.15;   % 0.15 - 0.25 m
pos_ss = position_amplitude + position_offset
tfinal = 38.0;

%%%% CONSTANTS %%%%
K_1 = -2.185;  % rad/Vs
K_2 = 0.0609;  % rad/rad
K_3 = -6.0885; % m / (s^2 * rad)
TAU = 0.0178;  % s

%%%% LAB 2 %%%%
OS_dec = 0.05;
T_SETTLING = 0.25;
ZETA_MAX = -log(OS_dec)/sqrt(pi^2 + log(OS_dec)^2);
THETA_MAX = rad2deg(acos(ZETA_MAX));
RE_S_MAX = -4 / T_SETTLING;

%%%% LAB 3 %%%%
OS_ball = 0.45;
T_SETTLING_ball = 8.5;
T_SETTLING_OVERALL = T_SETTLING_ball - T_SETTLING;
ZETA_BALL_MAX = -log(OS_ball)/sqrt(pi^2 + log(OS_ball)^2);
THETA_BALL_MAX = rad2deg(acos(ZETA_BALL_MAX));
RE_S_BALL_MAX = -4 / T_SETTLING_OVERALL;

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

% % 
% % C_2 = -7 * tf([1, 0.35], [1, 2.5]);
% % 
% % % SIMULINK PARAMETERS
% % D = c2d(C_2, T);
% % sim('general_SD_model.slx', tfinal);

% format long
% [NUM,DEN]=tfdata(D,'v');

% fprintf('float A2_0 = %.15f;\n', NUM(1));
% fprintf('float A2_1 = %.15f;\n', NUM(2));
% fprintf('float B2_0 = %.15f;\n', DEN(1));
% fprintf('float B2_1 = %.15f;\n', DEN(2));

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

output = []
begin_complex = 0;
RE_1 = RE_S_BALL_MAX;
COMPLEX2 = 0;
% for RE_1 = 0.05 * RE_S_BALL_MAX : 0.05 * RE_S_BALL_MAX : 4 * RE_S_BALL_MAX
    for RE_2 = 1 * RE_S_BALL_MAX : 0.5 * RE_S_BALL_MAX : 10 * RE_S_BALL_MAX
        for RE_3 = 1 * RE_S_BALL_MAX : 0.5 * RE_S_BALL_MAX : 20 * RE_S_BALL_MAX
%             for COMPLEX2 = begin_complex * -atan(deg2rad(THETA_BALL_MAX)) * RE_2: 0.05 * -atan(deg2rad(THETA_BALL_MAX)) * RE_2 : 1 * -atan(deg2rad(THETA_BALL_MAX)) * RE_2

NON_DOM_POLES_BALL_RE = RE_1;
NON_DOM_POLES_BALL_C = 0;
NON_DOM_POLES_BALL = conv([1 -NON_DOM_POLES_BALL_RE + NON_DOM_POLES_BALL_C * 1i], [1 -NON_DOM_POLES_BALL_RE - NON_DOM_POLES_BALL_C * 1i]);

NON_DOM_POLES_BALL_RE2 = RE_2;
NON_DOM_POLES_BALL_C2 = COMPLEX2;
NON_DOM_POLES_BALL2 = conv([1 -NON_DOM_POLES_BALL_RE2 + NON_DOM_POLES_BALL_C2 * 1i], [1 -NON_DOM_POLES_BALL_RE2 - NON_DOM_POLES_BALL_C2 * 1i]);

DOM_POLE = [1 -RE_3];

CP_DES = conv(NON_DOM_POLES_BALL, NON_DOM_POLES_BALL2);
CP_DES = conv(CP_DES, DOM_POLE).';
C_2_COEFF = linsolve(A, CP_DES);
C_2 = tf([C_2_COEFF(4), C_2_COEFF(5), C_2_COEFF(6)], [C_2_COEFF(1), C_2_COEFF(2), C_2_COEFF(3) 0]);
D_2 = c2d(C_2, T);
sim('general_SD_model.slx', tfinal);
y_cycle = y(25000:37500);
theta_cycle = theta(25000:37500);
theta_max = max(abs(min(theta_cycle)), abs(max(theta_cycle)));
os_perc = (max(y_cycle) - pos_ss) / position_amplitude * 100;
y_2settling = find(y_cycle < pos_ss - position_amplitude * 0.02 ...
    | y_cycle > pos_ss + position_amplitude * 0.02, 1, 'last') / 1000;
output = [output; RE_1, RE_2, COMPLEX2, RE_3, theta_max, os_perc, y_2settling;]
%             end
%             if complex_portion == -1
%                 break
%             end
%         end
    end
end

