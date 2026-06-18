%% LOAD FREQUENCY CONTROL (LFC) MASTER MATLAB SCRIPT
% Prof. Eng. Christopher Maina Muriithi
% This script collects key MATLAB examples for:
% 1. Numerical LFC calculations
% 2. Single-area LFC modeling
% 3. Root locus design
% 4. Bode stability analysis
% 5. Time-domain simulation with and without AGC
% 6. Two-area LFC state-space scaffold
%
% Notes:
% - Requires Control System Toolbox
% - Units should be used consistently
% - Some examples are simplified for tutorial purposes

clc; clear; close all;

fprintf('==============================================\n');
fprintf(' LOAD FREQUENCY CONTROL (LFC) MASTER SCRIPT\n');
fprintf('==============================================\n\n');

%% -------------------------------------------------
% SECTION 1: BASIC NUMERICAL CALCULATIONS
% --------------------------------------------------
fprintf('SECTION 1: BASIC NUMERICAL CALCULATIONS\n\n');

% Example 1: Steady-state frequency deviation
PL = 50;      % MW load increase
D  = 10;      % MW/Hz load damping
invR = 15;    % MW/Hz governor characteristic

df = -PL/(D + invR);
f_nom = 50;   % nominal frequency in Hz
f_new = f_nom + df;

fprintf('Example 1: Steady-state frequency deviation\n');
fprintf('Load increase = %.2f MW\n', PL);
fprintf('Frequency deviation = %.3f Hz\n', df);
fprintf('New operating frequency = %.3f Hz\n\n', f_new);

% Example 2: Generator sharing
nGen = 2;                       % identical generators
Pg_total = -invR * df;          % total primary control response
Pg_each = Pg_total / nGen;

fprintf('Example 2: Generator sharing\n');
fprintf('Total generation increase = %.2f MW\n', Pg_total);
fprintf('Each generator contributes = %.2f MW\n\n', Pg_each);

% Example 3: ACE calculation
Ptie = 50;      % MW
B_ace = 20;     % MW per 0.1 Hz
df_ace = -0.2;  % Hz
df_units = df_ace / 0.1;
ACE = Ptie + B_ace * df_units;

fprintf('Example 3: ACE calculation\n');
fprintf('ACE = %.2f MW\n\n', ACE);

% Example 4: Tie-line rate of change
T12_num = 0.1;       % pu
df1_num = 0.1;       % Hz
df2_num = -0.05;     % Hz
dPtie_dt = 2*pi*T12_num*(df1_num - df2_num);

fprintf('Example 4: Tie-line power rate of change\n');
fprintf('d(Delta P_tie)/dt = %.5f pu/s\n\n', dPtie_dt);

% Example 5: Area frequency response characteristic
D_beta = 12;
invR_beta = 18;
beta = D_beta + invR_beta;

fprintf('Example 5: Area frequency response characteristic\n');
fprintf('beta = %.2f MW/Hz\n\n', beta);

% Example 6: Effect of droop change
PL_drop = 50;
D_drop = 10;
invR_old = 10;
invR_new = 20;   % halving R doubles 1/R

df_old = -PL_drop/(D_drop + invR_old);
df_new = -PL_drop/(D_drop + invR_new);

fprintf('Example 6: Effect of halving droop R\n');
fprintf('Original frequency deviation = %.3f Hz\n', df_old);
fprintf('New frequency deviation      = %.3f Hz\n\n', df_new);

%% -------------------------------------------------
% SECTION 2: SINGLE-AREA LFC MODEL
% --------------------------------------------------
fprintf('SECTION 2: SINGLE-AREA LFC MODEL\n\n');

s = tf('s');

% Parameters
Tg = 0.2;    % governor time constant (s)
Tt = 0.5;    % turbine time constant (s)
H  = 5;      % inertia constant (s)
D  = 0.8;    % load damping
R  = 2.4;    % droop constant
Ki = 0.2;    % integral controller gain
Bf = D + 1/R; % frequency bias

% Component models
Gg = 1/(1 + Tg*s);        % Governor
Gt = 1/(1 + Tt*s);        % Turbine
Gp = 1/(2*H*s + D);       % Generator-load model

Gplant = Gg * Gt * Gp;

fprintf('Single-area transfer functions:\n');
disp('Governor Gg(s) = '); Gg;
disp('Turbine Gt(s) = '); Gt;
disp('Power system Gp(s) = '); Gp;

%% -------------------------------------------------
% SECTION 3: ROOT LOCUS DESIGN
% --------------------------------------------------
fprintf('SECTION 3: ROOT LOCUS DESIGN\n\n');

% AGC loop transfer function
G_rlocus = (Gg * Gt * Gp * (1/R)) / s;

figure;
rlocus(G_rlocus);
sgrid(0.5, []);
title('Root Locus of Single-Area LFC with Integral AGC');
grid on;

fprintf('Root locus plotted.\n');
fprintf('Use rlocfind(G_rlocus) interactively if needed.\n\n');

%% -------------------------------------------------
% SECTION 4: BODE AND STABILITY MARGINS
% --------------------------------------------------
fprintf('SECTION 4: BODE STABILITY ANALYSIS\n\n');

L = Ki * (Gg * Gt * Gp * (1/R)) / s;

figure;
margin(L);
grid on;
title('Bode Plot and Stability Margins of AGC Loop');

[Gm, Pm, Wcg, Wcp] = margin(L);

fprintf('Gain margin (absolute) = %.4f\n', Gm);
fprintf('Gain margin (dB)       = %.4f dB\n', 20*log10(Gm));
fprintf('Phase margin           = %.4f deg\n', Pm);
fprintf('Gain crossover freq    = %.4f rad/s\n', Wcg);
fprintf('Phase crossover freq   = %.4f rad/s\n\n', Wcp);

%% -------------------------------------------------
% SECTION 5: SINGLE-AREA RESPONSE WITH AND WITHOUT AGC
% --------------------------------------------------
fprintf('SECTION 5: SINGLE-AREA STEP RESPONSE\n\n');

% Primary control only
CL_primary = feedback(Gplant, 1/R);

% Primary + AGC
Hfb = 1/R + Ki*Bf/s;
CL_agc = feedback(Gplant, Hfb);

% Load step disturbance
t = 0:0.01:40;
u = -0.01 * ones(size(t));   % negative sign: load increase causes frequency drop

y_primary = lsim(CL_primary, u, t);
y_agc = lsim(CL_agc, u, t);

figure;
plot(t, y_primary, 'b', 'LineWidth', 1.6); hold on;
plot(t, y_agc, 'r', 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('\Delta f');
title('Single-Area LFC Response');
legend('Primary control only', 'With AGC', 'Location', 'Best');

fprintf('Single-area simulation completed.\n');
fprintf('Primary control leaves steady-state error; AGC reduces/removes it.\n\n');

%% -------------------------------------------------
% SECTION 6: INTERACTIVE ROOT LOCUS GAIN SELECTION
% --------------------------------------------------
fprintf('SECTION 6: OPTIONAL INTERACTIVE ROOT LOCUS SELECTION\n\n');
fprintf('Uncomment the next lines to use interactive gain selection.\n\n');

% figure;
% rlocus(G_rlocus);
% sgrid(0.5, []);
% [K_selected, poles_selected] = rlocfind(G_rlocus);
% fprintf('Selected K = %.4f\n', K_selected);
% disp('Selected closed-loop poles:');
% disp(poles_selected);

%% -------------------------------------------------
% SECTION 7: TWO-AREA LFC STATE-SPACE SCAFFOLD
% --------------------------------------------------
fprintf('SECTION 7: TWO-AREA LFC STATE-SPACE MODEL\n\n');

clc; clear; close all;

% -------------------------------------------------
% PARAMETERS
% -------------------------------------------------
Tg1 = 0.2;   Tt1 = 0.5;   H1 = 5.0;   D1 = 0.8;   R1 = 2.4;
Tg2 = 0.25;  Tt2 = 0.6;   H2 = 4.5;   D2 = 0.9;   R2 = 2.5;
T12 = 0.08;

B1 = D1 + 1/R1;
B2 = D2 + 1/R2;

Ki1 = 0.18;
Ki2 = 0.16;

% -------------------------------------------------
% STATE VECTOR
% x = [df1 Pm1 Pg1 df2 Pm2 Pg2 Ptie xi1 xi2]'
%
% df1   = frequency deviation area 1
% Pm1   = turbine mechanical power area 1
% Pg1   = governor output area 1
% df2   = frequency deviation area 2
% Pm2   = turbine mechanical power area 2
% Pg2   = governor output area 2
% Ptie  = tie-line power deviation
% xi1   = integral of ACE1
% xi2   = integral of ACE2
% -------------------------------------------------

A = zeros(9,9);
B = zeros(9,2);

% -------------------------------------------------
% AREA 1 DYNAMICS
% -------------------------------------------------
% d(df1)/dt
A(1,1) = -D1/(2*H1);
A(1,2) =  1/(2*H1);
A(1,7) = -1/(2*H1);      % tie-line appears as export/load to area 1

% d(Pm1)/dt
A(2,2) = -1/Tt1;
A(2,3) =  1/Tt1;

% d(Pg1)/dt
A(3,1) = -1/(R1*Tg1);    % droop feedback
A(3,3) = -1/Tg1;
A(3,8) = -Ki1/Tg1;       % AGC action through xi1

% -------------------------------------------------
% AREA 2 DYNAMICS
% -------------------------------------------------
% d(df2)/dt
A(4,4) = -D2/(2*H2);
A(4,5) =  1/(2*H2);
A(4,7) =  1/(2*H2);      % opposite sign of tie-line for area 2

% d(Pm2)/dt
A(5,5) = -1/Tt2;
A(5,6) =  1/Tt2;

% d(Pg2)/dt
A(6,4) = -1/(R2*Tg2);
A(6,6) = -1/Tg2;
A(6,9) = -Ki2/Tg2;

% -------------------------------------------------
% TIE-LINE DYNAMICS
% d(Ptie)/dt = 2*pi*T12*(df1 - df2)
% -------------------------------------------------
A(7,1) =  2*pi*T12;
A(7,4) = -2*pi*T12;

% -------------------------------------------------
% AGC INTEGRATOR STATES
% xi1_dot = ACE1 = B1*df1 + Ptie
% xi2_dot = ACE2 = B2*df2 - Ptie
% -------------------------------------------------
A(8,1) = B1;
A(8,7) = 1;

A(9,4) = B2;
A(9,7) = -1;

% -------------------------------------------------
% INPUTS: LOAD DISTURBANCES
% u(:,1) = Delta PL1
% u(:,2) = Delta PL2
% -------------------------------------------------
B(1,1) = -1/(2*H1);   % load disturbance in area 1
B(4,2) = -1/(2*H2);   % load disturbance in area 2

% -------------------------------------------------
% OUTPUTS
% y1 = df1, y2 = df2, y3 = Ptie
% -------------------------------------------------
C = [1 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0];

Dmat = zeros(3,2);

sys = ss(A,B,C,Dmat);

% -------------------------------------------------
% STABILITY CHECK
% -------------------------------------------------
disp('Eigenvalues of A:')
eigA = eig(A);
disp(eigA)

maxReal = max(real(eigA));
fprintf('Maximum real part of eigenvalues = %.6f\n', maxReal);

if maxReal < 0
    fprintf('System is asymptotically stable.\n');
elseif abs(maxReal) < 1e-8
    fprintf('System is marginally stable or very close to marginal.\n');
else
    fprintf('System is unstable.\n');
end

% -------------------------------------------------
% SIMULATION: STEP LOAD DISTURBANCE IN AREA 1
% -------------------------------------------------
t = 0:0.01:60;
u = zeros(length(t),2);
u(:,1) = 0.01;   % 0.01 pu load increase in area 1

y = lsim(sys,u,t);

% -------------------------------------------------
% PLOTS
% -------------------------------------------------
figure;
plot(t, y(:,1), 'LineWidth', 1.6); hold on;
plot(t, y(:,2), 'LineWidth', 1.6);
plot(t, y(:,3), 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('Response');
title('Corrected Two-Area LFC Responses');
legend('\Delta f_1', '\Delta f_2', '\Delta P_{tie}', 'Location', 'Best');

% -------------------------------------------------
% OPTIONAL: PLOT EACH RESPONSE SEPARATELY
% -------------------------------------------------
figure;
plot(t, y(:,1), 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('\Delta f_1');
title('Area 1 Frequency Deviation');

figure;
plot(t, y(:,2), 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('\Delta f_2');
title('Area 2 Frequency Deviation');

figure;
plot(t, y(:,3), 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('\Delta P_{tie}');
title('Tie-Line Power Deviation');
%% -------------------------------------------------
% SECTION 8: SUMMARY
% --------------------------------------------------
fprintf('SECTION 8: SUMMARY\n\n');
fprintf('This master script has executed:\n');
fprintf('1. Basic numerical calculations\n');
fprintf('2. Single-area transfer function modeling\n');
fprintf('3. Root locus analysis\n');
fprintf('4. Bode/margin analysis\n');
fprintf('5. Single-area step response with and without AGC\n');
fprintf('6. Two-area state-space scaffold simulation\n\n');

fprintf('End of LFC master MATLAB script.\n');