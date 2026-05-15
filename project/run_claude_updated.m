clf;
clear all; clc;
%% active suspension model used in butsuen
% for LQR control extended for h-infinity synthesis
% states:
%   x1 = zs-zu (rattle space)
%   x2 = zs_dot (body velocity)
%   x3 = zu-zr (tire displacement)
%   x4 = zu_dot (tire velocity)
% inputs:
%   u: actuator force
%   r_dot: road velocity
% outputs:
%   y1 = zs_dot_dot (body acceleration)
%   y2 = zs-zu (=x1) (rattle space)
% virtual outputs:
%   z1 = zs_dot_dot (body acceleration)
%   z2 = zs-zu (=x1) (rattle space)
%% define parameters
params = 'nom';
if strcmp(params,'bd')
    % Physical parameters (taken from bd example)
    ms = 300;    % kg
    mu = 60;     % kg
    bs = 1000;   % N/m/s
    ks = 16000 ; % N/m
    kt = 190000; % N/m
    bt = 0;      % N/m/s
elseif strcmp(params,'nom')
    % Physical parameters (taken from 1989 br)
    ms = 240;    % kg
    mu = 36;     % kg
    bs = 1000;   % N/m/s
    ks = 16000 ; % N/m
    kt = 160000; % N/m
    bt = 0;      % N/m/s
elseif strcmp(params,'suv')
    % Physical parameters
    ms = 600;    % kg
    mu = 60;     % kg
    bs = 3000;   % N/m/s
    ks = 20000 ; % N/m
    kt = 180000; % N/m
    bt = 0;      % N/m/s
elseif strcmp(params,'truck')
    % Physical parameters
    ms = 4000;    % kg
    mu = 200;     % kg
    bs = 8000;   % N/m/s
    ks = 150000 ; % N/m
    kt = 800000; % N/m
    bt = 0;      % N/m/s
end

% system dynamics
A = [0, 1, 0, -1;...
    -ks/ms, -bs/ms, 0, bs/ms;...
    0, 0, 0, 1;...
    ks/mu, bs/mu, -kt/mu, -(bs+bt)/mu];
B = [0;...
    1/ms;...
    0;...
    -1/mu];
L = [0;...
    0;...
    -1;...
    0];
B_x = [B,L];
C = [  A(2,:);...
    1, 0, 0, 0];
D = [1/ms, 0;...
    0,    0];
quartercar = ss(A, B_x, C, D);
quartercar.StateName = {'rattle space'; 'body velocity'; 'tire deflection'; 'tire velocity'};
quartercar.InputName = {'fs','r_dot'};
quartercar.OutputName = {'body acceleration'; 'rattle space'};

%% lqr design
ro1 = 400; ro2 = 400; ro3 = 16; ro4 = 16;
Q = [ks^2/ms^2+ro1, bs*ks/ms^2,    0,   -bs*ks/ms^2;...
    bs*ks/ms^2,    bs^2/ms^2+ro2, 0,   -bs^2/ms^2;...
    0,             0,             ro3, 0;...
    -bs*ks/ms^2,   -bs^2/ms^2,    0,   bs^2/ms^2+ro4];
N = [-ks/ms^2;...
    -bs/ms^2;...
    0;
    bs/ms^2];
R = 1/ms^2;
K_lqr = lqr(quartercar(:,1), Q, R, N);
quartercar_cl_lqr = ss(A-B*K_lqr,L,C,D(:,2));
quartercar_cl_lqr.StateName = {'rattle space'; 'body velocity'; 'tire deflection'; 'tire velocity'};
quartercar_cl_lqr.InputName = {'r_dot'};
quartercar_cl_lqr.OutputName = {'body acceleration'; 'rattle space'};
%% generate road profile at nominal speed V=25 m/s (used for original plots)
% roadprofile.m must have u0=25 and q=100
u0 = 25;
roadprofile;
dt     = t(2) - t(1);
rdot_raw = [diff(hsum)./dt, 0];
fs_sig = 1/dt;
[b_lp,a_lp] = butter(4, 50/(fs_sig/2), 'low');
rdot_iso = filtfilt(b_lp, a_lp, rdot_raw);

[y_ol_a,~] = lsim(quartercar('body acceleration','r_dot'), rdot_iso, t);
[y_ol_s,~] = lsim(quartercar('rattle space','r_dot'),      rdot_iso, t);
rms_a = rms(y_ol_a);
rms_s = rms(y_ol_s);
rms_r = rms(rdot_iso);

fprintf('OL accel RMS:  %.5f m/s^2\n', rms_a);
fprintf('OL rattle RMS: %.5f m\n',     rms_s);
fprintf('Road vel RMS:  %.5f m/s\n',   rms_r);
%% hinf — nominal design at V=25 m/s
beta   = 0.05;
V      = 25;   % nominal design speed — must match roadprofile.m u0
tighten = 4;

Wroad = tf(rms_r * sqrt(2), [1, 0.1]);
Wroad.u = 'd1'; Wroad.y = 'r_dot';

Act = tf(1,1);
Act.InputName = 'u'; Act.OutputName = 'fs';

Wact = 0.05 * tf([1,5],[1,80]);
Wact.u = 'u'; Wact.y = 'e1';

Wd2 = ss(0.01); Wd2.u = 'd2'; Wd2.y = 'Wd2';
Wd3 = ss(0.01); Wd3.u = 'd3'; Wd3.y = 'Wd3';

Wab = tighten*((1-beta)/rms_a) * tf([1/8, 1],[1/0.8, 1]);
Wab.u = 'body acceleration'; Wab.y = 'e2';

Wsd = tighten*(beta/rms_s) * tf([1/8, 1],[1/80, 1]);
Wsd.u = 'rattle space'; Wsd.y = 'e3';

sdmeas = sumblk('y1 = rattle space + Wd2');
abmeas = sumblk('y2 = body acceleration + Wd3');
ICinputs  = {'d1';'d2';'d3';'u'};
ICoutputs = {'e1';'e2';'e3';'y1';'y2'};

qcaric = connect(quartercar(1:2,:), Act, Wroad, Wact, Wab, Wsd, ...
    Wd2, Wd3, sdmeas, abmeas, ICinputs, ICoutputs);

% K_hinf(:,:,1) corresponds to V=25 m/s (nominal)
[K_hinf, ~, gamma] = hinfsyn(qcaric, 2, 1);
fprintf('gamma (V=25): %.4f\n', gamma);

K_hinf.InputName  = {'rattle space','body acceleration'};
K_hinf.OutputName = {'u'};

quartercar_cl_hinf = connect(quartercar, Act, K_hinf, ...
    'r_dot', {'body acceleration';'rattle space'});
%% bodes
figure(1);
bodemag(quartercar({'body acceleration';'rattle space'},'r_dot') , 'b--');
hold on
bodemag(quartercar_cl_lqr({'body acceleration';'rattle space'},'r_dot'), 'r');
hold on
bodemag(quartercar_cl_hinf({'body acceleration';'rattle space'},'r_dot') , 'k');
grid minor
legend('Open-loop','LQR','hinf V=25','location','SouthEast')
title('Bodes');
%% time simulations at V=25 (original — unchanged)
[y_ol, ~]      = lsim(quartercar('body acceleration','r_dot'), rdot_iso, t);
[y_lqr, ~]     = lsim(quartercar_cl_lqr('body acceleration','r_dot'), rdot_iso, t);
[y_hinf, ~]    = lsim(quartercar_cl_hinf('body acceleration','r_dot'), rdot_iso, t);
[y_ol_sd, ~]   = lsim(quartercar('rattle space','r_dot'), rdot_iso, t);
[y_lqr_sd, ~]  = lsim(quartercar_cl_lqr('rattle space','r_dot'), rdot_iso, t);
[y_hinf_sd, ~] = lsim(quartercar_cl_hinf('rattle space','r_dot'), rdot_iso, t);
%% performance metrics (original — unchanged)
disp('gamma='); disp(gamma);

rms_ol    = rms(y_ol);
rms_lqr   = rms(y_lqr);
rms_hinf  = rms(y_hinf);

fprintf('\n--- Performance Results (RMS Acceleration) ---\n');
fprintf('Open-Loop:  %.4f m/s^2\n', rms_ol);
fprintf('LQR:        %.4f m/s^2\n', rms_lqr);
fprintf('H-Infinity: %.4f m/s^2\n', rms_hinf);

rms_ol_sd   = rms(y_ol_sd);
rms_lqr_sd  = rms(y_lqr_sd);
rms_hinf_sd = rms(y_hinf_sd);

fprintf('\n--- Performance Results (RMS Rattle Space) ---\n');
fprintf('Open-Loop:  %.4f m\n', rms_ol_sd);
fprintf('LQR:        %.4f m\n', rms_lqr_sd);
fprintf('H-Infinity: %.4f m\n', rms_hinf_sd);

% Visualization (original — unchanged)
figure(2);
subplot(3,1,1);
plot(t, hsum*100, 'Color', [0.5 0.5 0.5]);
title('ISO 8608 Road Profile (Input) — V=25 m/s');
ylabel('Displacement [cm]'); grid on;

subplot(3,1,2);
plot(t, y_ol, 'b--', 'LineWidth', 0.5); hold on;
plot(t, y_lqr, 'r', 'LineWidth', 1);
plot(t, y_hinf, 'k', 'LineWidth', 1);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); legend('OL', 'LQR', 'Hinf'); grid on;

subplot(3,1,3);
plot(t, y_ol_sd*100, 'b--', 'LineWidth', 0.5); hold on;
plot(t, y_lqr_sd*100, 'r', 'LineWidth', 1);
plot(t, y_hinf_sd*100, 'k', 'LineWidth', 1);
title('Rattle Space (Suspension Travel)');
ylabel('Travel [cm]'); xlabel('Time [s]'); grid on;

figure(3);
plot(t, y_ol, 'b', 'LineWidth', 1); hold on;
plot(t, y_hinf, 'k', 'LineWidth', 1);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); legend('OL', 'Hinf'); grid on;

figure(4);
plot(t, y_ol_sd*100, 'b', 'LineWidth', 1); hold on;
plot(t, y_hinf_sd*100, 'k', 'LineWidth', 1);
title('Rattle Space');
ylabel('cm'); legend('OL', 'Hinf'); grid on;

%% MULTI-SPEED H-INFINITY SYNTHESIS  (25:5:50 m/s)
% For each speed Vi we:
%   1. Generate a road profile at Vi  (roadprofile.m u0 must be set to Vi)
%   2. Compute open-loop RMS at Vi to calibrate weights
%   3. Synthesise a dedicated H-inf controller K_hinf_all{i}
%   4. Store the matching closed-loop system cl_hinf_all{i}
%
% NOTE: roadprofile.m is called inside the loop with u0 overridden here.
%       Make sure roadprofile.m reads u0 from the workspace (default) OR
%       replace the u0 line in roadprofile.m with:  if ~exist('u0','var'), u0=25; end

V_grid = 25:5:50;          % [25 30 35 40 45 50]  m/s
nV     = length(V_grid);

K_hinf_all  = cell(nV, 1); % cell array of controllers
cl_hinf_all = cell(nV, 1); % cell array of closed-loop systems
gamma_all   = zeros(nV, 1);
rms_r_all   = zeros(nV, 1); % road velocity RMS at each speed (for Wroad)

fprintf('\n=== Synthesising H-inf controllers for V = %d:%d:%d m/s ===\n', ...
    V_grid(1), V_grid(2)-V_grid(1), V_grid(end));

for i = 1:nV
    Vi = V_grid(i);
    fprintf('\n--- V = %d m/s ---\n', Vi);

    % --- 1. Generate road profile at this speed ---
    % Override u0 before calling roadprofile so the time vector scales correctly.
    % roadprofile.m must use the workspace variable u0 (not a hard-coded value).
    u0 = Vi;  %#ok<NASGU>  passed into roadprofile.m via base workspace
    roadprofile;                           % produces hsum, t, Lfin

    dt_i     = t(2) - t(1);
    rdot_raw_i = [diff(hsum)./dt_i, 0];
    fs_i     = 1/dt_i;
    [b_i,a_i] = butter(4, 50/(fs_i/2), 'low');
    rdot_i   = filtfilt(b_i, a_i, rdot_raw_i);

    % --- 2. Open-loop RMS at Vi for weight calibration ---
    [ya_i,~] = lsim(quartercar('body acceleration','r_dot'), rdot_i, t);
    [ys_i,~] = lsim(quartercar('rattle space','r_dot'),      rdot_i, t);
    rms_a_i  = rms(ya_i);
    rms_s_i  = rms(ys_i);
    rms_r_i  = rms(rdot_i);
    rms_r_all(i) = rms_r_i;

    fprintf('  OL accel RMS: %.5f m/s^2 | rattle RMS: %.5f m | road vel RMS: %.5f m/s\n',...
        rms_a_i, rms_s_i, rms_r_i);

    % --- 3. Build speed-specific weights ---
    Wroad_i = tf(rms_r_i * sqrt(2), [1, 0.1]);
    Wroad_i.u = 'd1'; Wroad_i.y = 'r_dot';

    Act_i = tf(1,1);
    Act_i.InputName = 'u'; Act_i.OutputName = 'fs';

    Wact_i = 0.05 * tf([1,5],[1,80]);
    Wact_i.u = 'u'; Wact_i.y = 'e1';

    Wd2_i = ss(0.01); Wd2_i.u = 'd2'; Wd2_i.y = 'Wd2';
    Wd3_i = ss(0.01); Wd3_i.u = 'd3'; Wd3_i.y = 'Wd3';

    Wab_i = tighten*((1-beta)/rms_a_i) * tf([1/8, 1],[1/0.8, 1]);
    Wab_i.u = 'body acceleration'; Wab_i.y = 'e2';

    Wsd_i = tighten*(beta/rms_s_i) * tf([1/8, 1],[1/80, 1]);
    Wsd_i.u = 'rattle space'; Wsd_i.y = 'e3';

    sdmeas_i = sumblk('y1 = rattle space + Wd2');
    abmeas_i = sumblk('y2 = body acceleration + Wd3');

    qcaric_i = connect(quartercar(1:2,:), Act_i, Wroad_i, Wact_i, Wab_i, Wsd_i, ...
        Wd2_i, Wd3_i, sdmeas_i, abmeas_i, ICinputs, ICoutputs);

    % --- 4. Synthesise ---
    [Ki, ~, gi] = hinfsyn(qcaric_i, 2, 1);
    Ki.InputName  = {'rattle space','body acceleration'};
    Ki.OutputName = {'u'};

    K_hinf_all{i}  = Ki;
    gamma_all(i)   = gi;
    cl_hinf_all{i} = connect(quartercar, Act_i, Ki, ...
        'r_dot', {'body acceleration';'rattle space'});

    fprintf('  gamma = %.4f\n', gi);
end

fprintf('\n=== Synthesis complete ===\n');
fprintf('Speed (m/s):  '); fprintf('%6d ', V_grid);      fprintf('\n');
fprintf('Gamma:        '); fprintf('%6.3f ', gamma_all); fprintf('\n');

%% CROSS-SPEED VALIDATION
% Motivation: show that a controller designed at V=25 degrades at V=50,
% while the controller designed at V=50 performs best at V=50.
% For each test speed in V_grid we simulate:
%   (a) Open-loop
%   (b) H-inf designed at V=25  (K_hinf_all{1})  — the "mismatched" controller
%   (c) H-inf designed at Vi    (K_hinf_all{i})  — the "matched"   controller
% and record RMS acceleration and rattle space.

fprintf('\n=== Cross-speed validation ===\n');
fprintf('%-10s %-12s %-18s %-18s %-14s %-14s\n', ...
    'Speed', 'OL accel', 'Hinf@25 accel', 'HinfMatched accel', ...
    'Hinf@25 rattle','HinfMatched rattle');

rms_accel_ol      = zeros(nV,1);
rms_accel_hinf25  = zeros(nV,1);   % V=25 controller tested at all speeds
rms_accel_matched = zeros(nV,1);   % speed-matched controller
rms_rattle_ol     = zeros(nV,1);
rms_rattle_hinf25 = zeros(nV,1);
rms_rattle_matched= zeros(nV,1);

% Retrieve the V=25 controller (first in grid)
K25  = K_hinf_all{1};
cl25 = cl_hinf_all{1};  % closed-loop built with K@25 — we rebuild below per speed

for i = 1:nV
    Vi = V_grid(i);

    % Generate road at test speed Vi
    u0 = Vi;  %#ok<NASGU>
    roadprofile;
    dt_i = t(2)-t(1);
    rdot_raw_i = [diff(hsum)./dt_i, 0];
    fs_i = 1/dt_i;
    [b_i,a_i] = butter(4, 50/(fs_i/2), 'low');
    rdot_i = filtfilt(b_i, a_i, rdot_raw_i);

    % Rebuild closed-loop for K@25 applied at speed Vi
    % (K@25 is fixed; only the road input changes)
    cl_K25_at_Vi = connect(quartercar, Act, K25, ...
        'r_dot', {'body acceleration';'rattle space'});

    % Open-loop
    [ya_ol,~]  = lsim(quartercar('body acceleration','r_dot'), rdot_i, t);
    [ys_ol,~]  = lsim(quartercar('rattle space','r_dot'),      rdot_i, t);

    % H-inf @ V=25, tested at Vi
    [ya_25,~]  = lsim(cl_K25_at_Vi('body acceleration','r_dot'), rdot_i, t);
    [ys_25,~]  = lsim(cl_K25_at_Vi('rattle space','r_dot'),      rdot_i, t);

    % H-inf matched (designed at Vi), tested at Vi
    [ya_m,~]   = lsim(cl_hinf_all{i}('body acceleration','r_dot'), rdot_i, t);
    [ys_m,~]   = lsim(cl_hinf_all{i}('rattle space','r_dot'),      rdot_i, t);

    rms_accel_ol(i)       = rms(ya_ol);
    rms_accel_hinf25(i)   = rms(ya_25);
    rms_accel_matched(i)  = rms(ya_m);
    rms_rattle_ol(i)      = rms(ys_ol);
    rms_rattle_hinf25(i)  = rms(ys_25);
    rms_rattle_matched(i) = rms(ys_m);

    fprintf('%-10d %-12.4f %-18.4f %-18.4f %-14.4f %-14.4f\n', Vi, ...
        rms_accel_ol(i), rms_accel_hinf25(i), rms_accel_matched(i), ...
        rms_rattle_hinf25(i), rms_rattle_matched(i));
end

% --- Percentage error of H-inf@25 vs matched controller ---
% Positive = H-inf@25 is worse (higher RMS) than matched
pct_accel  = 100*(rms_accel_hinf25  - rms_accel_matched)  ./ rms_accel_matched;
pct_rattle = 100*(rms_rattle_hinf25 - rms_rattle_matched) ./ rms_rattle_matched;

fprintf('\n--- Penalty of using H-inf@25 instead of matched controller ---\n');
fprintf('%-10s %-20s %-20s\n','Speed','Accel penalty (%%)','Rattle penalty (%%)');
for i = 1:nV
    fprintf('%-10d %-20.2f %-20.2f\n', V_grid(i), pct_accel(i), pct_rattle(i));
end
%% CROSS-SPEED FIGURES
% Figure 5 — RMS Acceleration vs Speed
figure(5);
plot(V_grid, rms_accel_ol,      'b--o', 'LineWidth', 1.5, 'MarkerSize', 7); hold on;
plot(V_grid, rms_accel_hinf25,  'r-s',  'LineWidth', 1.5, 'MarkerSize', 7);
plot(V_grid, rms_accel_matched, 'k-^',  'LineWidth', 1.5, 'MarkerSize', 7);
xlabel('Vehicle Speed (m/s)');
ylabel('RMS Body Acceleration (m/s^2)');
title('Comfort Performance vs Speed');
legend('Open-loop', 'H-inf designed @ V=25', 'H-inf matched to speed', ...
    'Location','NorthWest');
grid on;
% Annotate the key comparison at V=50
[~, idx50] = max(V_grid == 50);
text(50, rms_accel_hinf25(idx50)*1.02, ...
    sprintf('%.4f', rms_accel_hinf25(idx50)), ...
    'Color','r','FontSize',9,'HorizontalAlignment','center');
text(50, rms_accel_matched(idx50)*0.97, ...
    sprintf('%.4f', rms_accel_matched(idx50)), ...
    'Color','k','FontSize',9,'HorizontalAlignment','center');

% Figure 6 — RMS Rattle Space vs Speed
figure(6);
plot(V_grid, rms_rattle_ol*100,      'b--o', 'LineWidth', 1.5, 'MarkerSize', 7); hold on;
plot(V_grid, rms_rattle_hinf25*100,  'r-s',  'LineWidth', 1.5, 'MarkerSize', 7);
plot(V_grid, rms_rattle_matched*100, 'k-^',  'LineWidth', 1.5, 'MarkerSize', 7);
xlabel('Vehicle Speed (m/s)');
ylabel('RMS Rattle Space (cm)');
title('Handling Performance vs Speed');
legend('Open-loop', 'H-inf designed @ V=25', 'H-inf matched to speed', ...
    'Location','NorthWest');
grid on;
% Figure 7 — Percentage penalty of mismatched controller vs speed
figure(7);
bar(V_grid, [pct_accel, pct_rattle], 'grouped');
xlabel('Vehicle Speed (m/s)');
ylabel('Performance Penalty vs Matched Controller (%)');
title('Cost of Using a Fixed H-inf Controller (Designed at V=25 m/s)');
legend('Body Acceleration', 'Rattle Space', 'Location','NorthWest');
grid on;
yline(0, 'k--');
% A positive bar means H-inf@25 is worse than the matched design
% The V=25 bars should be near zero (same controller)
% The V=50 bars show the degradation — key result
%% Figure 8 — Time-domain comparison at V=50 m/s
%  Regenerate road at V=50 for time plots
u0 = 50;  %#ok<NASGU>
roadprofile;
dt_50 = t(2)-t(1);
rdot_raw_50 = [diff(hsum)./dt_50, 0];
[b50,a50] = butter(4, 50/(1/dt_50/2), 'low');
rdot_50 = filtfilt(b50, a50, rdot_raw_50);

cl_K25_at_50 = connect(quartercar, Act, K25, ...
    'r_dot', {'body acceleration';'rattle space'});

[ya_ol50,~]   = lsim(quartercar('body acceleration','r_dot'), rdot_50, t);
[ya_h25_50,~] = lsim(cl_K25_at_50('body acceleration','r_dot'), rdot_50, t);
[ya_hm50,~]   = lsim(cl_hinf_all{end}('body acceleration','r_dot'), rdot_50, t);

[ys_ol50,~]   = lsim(quartercar('rattle space','r_dot'), rdot_50, t);
[ys_h25_50,~] = lsim(cl_K25_at_50('rattle space','r_dot'), rdot_50, t);
[ys_hm50,~]   = lsim(cl_hinf_all{end}('rattle space','r_dot'), rdot_50, t);

t_plot = t(1:min(end, round(10/dt_50)));  % show first 10 seconds for clarity
idx_p  = 1:length(t_plot);

figure(8);
subplot(2,1,1);
plot(t_plot, ya_ol50(idx_p),   'b--', 'LineWidth',0.8); hold on;
plot(t_plot, ya_h25_50(idx_p), 'r',   'LineWidth',1.2);
plot(t_plot, ya_hm50(idx_p),   'k',   'LineWidth',1.2);
title('Body Acceleration at V=50 m/s');
ylabel('m/s^2');
legend(sprintf('OL  (%.4f m/s^2)', rms(ya_ol50)), ...
    sprintf('Hinf@25  (%.4f m/s^2)', rms(ya_h25_50(idx_p))), ...
    sprintf('Hinf@50  (%.4f m/s^2)', rms(ya_hm50(idx_p))), ...
    'Location','NorthEast');
grid on;

subplot(2,1,2);
plot(t_plot, ys_ol50(idx_p)*100,   'b--', 'LineWidth',0.8); hold on;

set(findall(0, 'type', 'figure'), 'WindowStyle', 'docked'); % dock all figures
plot(t_plot, ys_h25_50(idx_p)*100, 'r',   'LineWidth',1.2);
plot(t_plot, ys_hm50(idx_p)*100,   'k',   'LineWidth',1.2);
title('Rattle Space at V=50 m/s');
ylabel('cm'); xlabel('Time (s)');
legend(sprintf('OL  (%.4f cm)', rms(ys_ol50)*100), ...
    sprintf('Hinf@25  (%.4f cm)', rms(ys_h25_50(idx_p))*100), ...
    sprintf('Hinf@50  (%.4f cm)', rms(ys_hm50(idx_p))*100), ...
    'Location','NorthEast');
grid on;
sgtitle('Matched vs Mismatched H-inf Controller at V=50 m/s — Key Comparison');
%% LPV-H∞ DESIGN — Scheduling on Sprung Mass (Body Natural Frequency)
% Motivation:
%   Speed does NOT enter the plant matrices A,B,C,D — it only affects the
%   road disturbance character (Wroad).  Sprung mass ms DOES enter A and B
%   directly, making it a genuine LPV scheduling variable.
%
%   When passengers load/unload, ms changes from ~180 kg (driver only) to
%   ~400 kg (full load).  The body resonance shifts:
%       w_body = sqrt(ks/ms)  →  9.43 rad/s (light) to 6.32 rad/s (heavy)
%   A fixed H∞ controller designed at ms=240 has no formal guarantee at
%   ms=400.  The LPV scheduler continuously adapts.
%
% Approach:
%   1. Define a mass grid  ms_grid = [180, 220, 260, 300, 340, 400] kg
%   2. At each grid point rebuild the plant and synthesise a dedicated H∞
%      controller (same weight structure, calibrated to that plant's OL RMS)
%   3. At runtime interpolate the controller state-space matrices linearly
%      with the scheduling variable  rho = 1/ms  (affine in plant matrices)
%   4. Validate: simulate a mass-varying scenario and compare
%      Fixed H∞ (ms=240) vs LPV-H∞ (scheduled) vs Open-loop
%
% NOTE: roadprofile.m u0 is restored to 25 m/s for all LPV simulations
%       so that road conditions are held constant and only mass varies.
 
fprintf('\n\n=========================================================\n');
fprintf('  LPV-H-inf DESIGN — Scheduling on Sprung Mass\n');
fprintf('=========================================================\n');
 
%% --- Step 0: restore nominal road (V=25, same Gh0 as before) ---
u0 = 25;
roadprofile;
dt_lpv   = t(2)-t(1);
rdot_raw_lpv = [diff(hsum)./dt_lpv, 0];
fs_lpv   = 1/dt_lpv;
[b_lpv, a_lpv] = butter(4, 50/(fs_lpv/2), 'low');
rdot_lpv = filtfilt(b_lpv, a_lpv, rdot_raw_lpv);   % road input fixed for all LPV tests
 
%% --- Step 1: Mass grid and per-point H∞ synthesis ---
ms_grid  = [180, 220, 260, 300, 340, 400];   % kg  — realistic passenger loading range
nM       = length(ms_grid);
 
K_lpv_all   = cell(nM,1);   % H∞ controllers at each mass grid point
cl_lpv_all  = cell(nM,1);   % closed-loop systems at each mass grid point
gamma_lpv   = zeros(nM,1);
w_body_grid = zeros(nM,1);  % natural frequencies at grid points
 
fprintf('\n=== Synthesising H-inf controllers for mass grid ===\n');
fprintf('ms (kg):    '); fprintf('%7.0f ', ms_grid); fprintf('\n');
 
for i = 1:nM
    ms_i = ms_grid(i);
    w_body_grid(i) = sqrt(ks/ms_i);
    fprintf('\n--- ms = %d kg  (w_body = %.3f rad/s = %.3f Hz) ---\n', ...
        ms_i, w_body_grid(i), w_body_grid(i)/(2*pi));
 
    % Rebuild plant at this mass
    A_i = [0,      1,       0,  -1;
           -ks/ms_i, -bs/ms_i,  0,  bs/ms_i;
            0,      0,       0,   1;
            ks/mu,  bs/mu, -kt/mu, -(bs+bt)/mu];
    B_i   = [0; 1/ms_i; 0; -1/mu];
    B_x_i = [B_i, L];
    C_i   = [A_i(2,:); 1, 0, 0, 0];
    D_i   = [1/ms_i, 0; 0, 0];
 
    plant_i = ss(A_i, B_x_i, C_i, D_i);
    plant_i.StateName  = {'rattle space';'body velocity';'tire deflection';'tire velocity'};
    plant_i.InputName  = {'fs','r_dot'};
    plant_i.OutputName = {'body acceleration';'rattle space'};
 
    % Calibrate weights using OL RMS at this plant + nominal road
    [ya_i,~] = lsim(plant_i('body acceleration','r_dot'), rdot_lpv, t);
    [ys_i,~] = lsim(plant_i('rattle space','r_dot'),      rdot_lpv, t);
    rms_a_i  = rms(ya_i);
    rms_s_i  = rms(ys_i);
    rms_r_i  = rms(rdot_lpv);   % road is fixed, same for all
 
    fprintf('  OL accel: %.5f m/s^2 | rattle: %.6f m\n', rms_a_i, rms_s_i);
 
    % Build interconnection (same structure as nominal H∞)
    Wroad_i = tf(rms_r_i*sqrt(2), [1,0.1]);
    Wroad_i.u = 'd1'; Wroad_i.y = 'r_dot';
 
    Act_i = tf(1,1); Act_i.InputName = 'u'; Act_i.OutputName = 'fs';
 
    Wact_i = 0.05*tf([1,5],[1,80]);
    Wact_i.u = 'u'; Wact_i.y = 'e1';
 
    Wd2_i = ss(0.01); Wd2_i.u = 'd2'; Wd2_i.y = 'Wd2';
    Wd3_i = ss(0.01); Wd3_i.u = 'd3'; Wd3_i.y = 'Wd3';
 
    Wab_i = tighten*((1-beta)/rms_a_i) * tf([1/8,1],[1/0.8,1]);
    Wab_i.u = 'body acceleration'; Wab_i.y = 'e2';
 
    Wsd_i = tighten*(beta/rms_s_i) * tf([1/8,1],[1/80,1]);
    Wsd_i.u = 'rattle space'; Wsd_i.y = 'e3';
 
    sdmeas_i = sumblk('y1 = rattle space + Wd2');
    abmeas_i = sumblk('y2 = body acceleration + Wd3');
 
    qcaric_i = connect(plant_i(1:2,:), Act_i, Wroad_i, Wact_i, Wab_i, Wsd_i, ...
                       Wd2_i, Wd3_i, sdmeas_i, abmeas_i, ICinputs, ICoutputs);
 
    [Ki, ~, gi] = hinfsyn(qcaric_i, 2, 1);
    Ki.InputName  = {'rattle space','body acceleration'};
    Ki.OutputName = {'u'};
 
    K_lpv_all{i}  = Ki;
    gamma_lpv(i)  = gi;
    % Store closed-loop for the plant at ms_i
    cl_lpv_all{i} = connect(plant_i, Act_i, Ki, ...
                        'r_dot', {'body acceleration';'rattle space'});
 
    fprintf('  gamma = %.4f\n', gi);
end
 
fprintf('\n=== LPV synthesis complete ===\n');
fprintf('ms (kg):    '); fprintf('%7.0f ', ms_grid);    fprintf('\n');
fprintf('w_b (r/s):  '); fprintf('%7.3f ', w_body_grid);fprintf('\n');
fprintf('Gamma:      '); fprintf('%7.4f ', gamma_lpv);  fprintf('\n');
 
%% --- Step 2: Controller order check (required before interpolation) ---
fprintf('\n--- Controller orders ---\n');
orders = zeros(nM,1);
for i = 1:nM
    orders(i) = order(K_lpv_all{i});
    fprintf('  ms=%d kg: order = %d\n', ms_grid(i), orders(i));
end
if all(orders == orders(1))
    fprintf('  All controllers have equal order %d — interpolation valid.\n', orders(1));
else
    warning('Controllers have different orders. Interpolation may be unreliable.');
end
 
%% --- Step 3: LPV interpolation function (defined inline via anonymous fn) ---
% Scheduling variable: rho = ms  (linear interpolation on ms axis)
% Interpolates A_K, B_K, C_K, D_K matrices between grid points.
%
% Usage:  K_interp = lpv_interpolate(ms_val, ms_grid, K_lpv_all)
 
lpv_interpolate = @(ms_val) interpolate_controller(ms_val, ms_grid, K_lpv_all);
 
% Helper function defined at bottom of file — see local function section.
 
%% --- Step 4: Mass variation validation ---
% Test masses — includes off-grid points to test interpolation quality
ms_test = [180, 200, 220, 240, 260, 280, 300, 320, 340, 370, 400];
nTest   = length(ms_test);
 
rms_accel_ol_ms    = zeros(nTest,1);
rms_accel_fixed_ms = zeros(nTest,1);   % fixed H∞ designed at ms=240 (nominal)
rms_accel_lpv_ms   = zeros(nTest,1);   % LPV-scheduled H∞
rms_rattle_ol_ms   = zeros(nTest,1);
rms_rattle_fixed_ms= zeros(nTest,1);
rms_rattle_lpv_ms  = zeros(nTest,1);
stable_fixed_ms    = false(nTest,1);
stable_lpv_ms      = false(nTest,1);
 
% Fixed H∞ is K_hinf (designed at nominal ms, from original section)
Act_u = tf(1,1); Act_u.InputName='u'; Act_u.OutputName='fs';
 
fprintf('\n=== Mass variation validation ===\n');
fprintf('%-10s %-12s %-16s %-16s %-14s %-14s %-10s %-10s\n',...
    'ms (kg)','OL accel','Fixed-Hinf accel','LPV-Hinf accel',...
    'Fixed rattle','LPV rattle','Stab-Fix','Stab-LPV');
 
for i = 1:nTest
    ms_t = ms_test(i);
 
    % Rebuild plant at test mass
    A_t = [0,      1,       0,  -1;
           -ks/ms_t, -bs/ms_t,  0,  bs/ms_t;
            0,      0,       0,   1;
            ks/mu,  bs/mu, -kt/mu, -(bs+bt)/mu];
    B_t   = [0; 1/ms_t; 0; -1/mu];
    B_x_t = [B_t, L];
    C_t   = [A_t(2,:); 1, 0, 0, 0];
    D_t   = [1/ms_t, 0; 0, 0];
 
    plant_t = ss(A_t, B_x_t, C_t, D_t);
    plant_t.StateName  = {'rattle space';'body velocity';'tire deflection';'tire velocity'};
    plant_t.InputName  = {'fs','r_dot'};
    plant_t.OutputName = {'body acceleration';'rattle space'};
 
    % LPV controller scheduled at ms_t
    K_lpv_t = lpv_interpolate(ms_t);
 
    % Close both loops on the perturbed plant
    cl_fixed_t = connect(plant_t, Act_u, K_hinf, ...
                     'r_dot',{'body acceleration';'rattle space'});
    cl_lpv_t   = connect(plant_t, Act_u, K_lpv_t, ...
                     'r_dot',{'body acceleration';'rattle space'});
 
    % Stability check
    stable_fixed_ms(i) = all(real(pole(cl_fixed_t)) < 0);
    stable_lpv_ms(i)   = all(real(pole(cl_lpv_t))   < 0);
 
    % Simulate — use the fixed nominal road input (rdot_lpv)
    [ya_ol_t, ~]  = lsim(plant_t('body acceleration','r_dot'), rdot_lpv, t);
    [ys_ol_t, ~]  = lsim(plant_t('rattle space','r_dot'),      rdot_lpv, t);
 
    if stable_fixed_ms(i)
        [ya_fx,~] = lsim(cl_fixed_t('body acceleration','r_dot'), rdot_lpv, t);
        [ys_fx,~] = lsim(cl_fixed_t('rattle space','r_dot'),      rdot_lpv, t);
        rms_accel_fixed_ms(i)  = rms(ya_fx);
        rms_rattle_fixed_ms(i) = rms(ys_fx);
    else
        rms_accel_fixed_ms(i)  = NaN;
        rms_rattle_fixed_ms(i) = NaN;
    end
 
    if stable_lpv_ms(i)
        [ya_lv,~] = lsim(cl_lpv_t('body acceleration','r_dot'), rdot_lpv, t);
        [ys_lv,~] = lsim(cl_lpv_t('rattle space','r_dot'),      rdot_lpv, t);
        rms_accel_lpv_ms(i)  = rms(ya_lv);
        rms_rattle_lpv_ms(i) = rms(ys_lv);
    else
        rms_accel_lpv_ms(i)  = NaN;
        rms_rattle_lpv_ms(i) = NaN;
    end
 
    rms_accel_ol_ms(i)  = rms(ya_ol_t);
    rms_rattle_ol_ms(i) = rms(ys_ol_t);
 
    fprintf('%-10d %-12.4f %-16.4f %-16.4f %-14.4f %-14.4f %-10s %-10s\n',...
        ms_t, rms_accel_ol_ms(i), rms_accel_fixed_ms(i), rms_accel_lpv_ms(i),...
        rms_rattle_fixed_ms(i), rms_rattle_lpv_ms(i),...
        string(stable_fixed_ms(i)), string(stable_lpv_ms(i)));
end
 
% Percentage penalty of fixed H∞ vs LPV
pct_accel_lpv  = 100*(rms_accel_fixed_ms  - rms_accel_lpv_ms)  ./ rms_accel_lpv_ms;
pct_rattle_lpv = 100*(rms_rattle_fixed_ms - rms_rattle_lpv_ms) ./ rms_rattle_lpv_ms;
 
fprintf('\n--- Penalty of Fixed H-inf vs LPV-H-inf across mass range ---\n');
fprintf('  Positive = Fixed H-inf is worse (LPV wins)\n');
fprintf('%-10s %-22s %-22s\n','ms (kg)','Accel penalty (%%)','Rattle penalty (%%)');
for i = 1:nTest
    fprintf('%-10d %-22.2f %-22.2f\n', ms_test(i), pct_accel_lpv(i), pct_rattle_lpv(i));
end
 
%% --- Step 5: Natural frequency sweep plot ---
w_body_test = sqrt(ks ./ ms_test);
 
figure(9);
plot(w_body_test, rms_accel_ol_ms,    'b--o','LineWidth',1.5,'MarkerSize',7); hold on;
plot(w_body_test, rms_accel_fixed_ms, 'r-s', 'LineWidth',1.5,'MarkerSize',7);
plot(w_body_test, rms_accel_lpv_ms,   'k-^', 'LineWidth',1.5,'MarkerSize',7);
xlabel('Body Natural Frequency \omega_{body} (rad/s)');
ylabel('RMS Body Acceleration (m/s^2)');
title('Comfort vs Body Natural Frequency — Fixed H-inf vs LPV-H-inf');
legend('Open-loop','Fixed H-inf (ms=240)','LPV-H-inf (scheduled)','Location','Best');
grid on;
% Mark nominal design point
xline(sqrt(ks/ms), 'k--', sprintf('Design point ms=%dkg',ms),'LabelVerticalAlignment','top');
set(gca, 'XDir', 'reverse');  % or use ms_test as x-axis instead
figure(10);
plot(w_body_test, rms_rattle_ol_ms*100,    'b--o','LineWidth',1.5,'MarkerSize',7); hold on;
plot(w_body_test, rms_rattle_fixed_ms*100, 'r-s', 'LineWidth',1.5,'MarkerSize',7);
plot(w_body_test, rms_rattle_lpv_ms*100,   'k-^', 'LineWidth',1.5,'MarkerSize',7);
xlabel('Body Natural Frequency \omega_{body} (rad/s)');
ylabel('RMS Rattle Space (cm)');
title('Handling vs Body Natural Frequency — Fixed H-inf vs LPV-H-inf');
legend('Open-loop','Fixed H-inf (ms=240)','LPV-H-inf (scheduled)','Location','Best');
grid on;
xline(sqrt(ks/ms), 'k--', sprintf('Design point ms=%dkg',ms),'LabelVerticalAlignment','top');
 
%% --- Step 6: Pole migration plot (stability visualization) ---
figure(11);
hold on;
colors = lines(nM);
for i = 1:nM
    ms_i  = ms_grid(i);
    A_i   = [0, 1, 0, -1;
             -ks/ms_i, -bs/ms_i, 0, bs/ms_i;
              0, 0, 0, 1;
              ks/mu, bs/mu, -kt/mu, -(bs+bt)/mu];
    B_x_i = [[0;1/ms_i;0;-1/mu], L];
    C_i   = [A_i(2,:); 1,0,0,0];
    D_i   = [1/ms_i,0;0,0];
    plant_i = ss(A_i, B_x_i, C_i, D_i);
    plant_i.InputName = {'fs','r_dot'}; plant_i.OutputName = {'body acceleration';'rattle space'};
    Act_i2 = tf(1,1); Act_i2.InputName='u'; Act_i2.OutputName='fs';
    cl_i = connect(plant_i, Act_i2, K_lpv_all{i},'r_dot',{'body acceleration';'rattle space'});
    p = pole(cl_i);
    scatter(real(p), imag(p), 60, colors(i,:), 'filled','DisplayName',sprintf('ms=%dkg',ms_i));
end
xline(0,'k--','LineWidth',1.5);
xlabel('Real Part'); ylabel('Imaginary Part');
title('Closed-loop Pole Migration — LPV Controllers Across Mass Grid');
legend('Location','Best'); grid on;
xlim([-max(abs(real(pole(cl_lpv_all{1}))))*1.2, ...
       max(abs(real(pole(cl_lpv_all{1}))))*0.3]);
 
%% --- Step 7: Time-domain comparison at off-nominal mass (ms=380 kg, off-grid) ---
ms_offgrid = 380;   % loaded vehicle — between grid points 340 and 400
fprintf('\n--- Time-domain comparison at ms=%d kg (off-grid) ---\n', ms_offgrid);
 
A_og   = [0,1,0,-1; -ks/ms_offgrid,-bs/ms_offgrid,0,bs/ms_offgrid; 0,0,0,1; ks/mu,bs/mu,-kt/mu,-(bs+bt)/mu];
B_og   = [0;1/ms_offgrid;0;-1/mu];
plant_og = ss(A_og,[B_og,L],[A_og(2,:);1,0,0,0],[1/ms_offgrid,0;0,0]);
plant_og.InputName={'fs','r_dot'}; plant_og.OutputName={'body acceleration';'rattle space'};
 
K_lpv_og = lpv_interpolate(ms_offgrid);
 
Act_og = tf(1,1); Act_og.InputName='u'; Act_og.OutputName='fs';
cl_fixed_og = connect(plant_og, Act_og, K_hinf,    'r_dot',{'body acceleration';'rattle space'});
cl_lpv_og   = connect(plant_og, Act_og, K_lpv_og,  'r_dot',{'body acceleration';'rattle space'});
 
[ya_ol_og, ~]  = lsim(plant_og('body acceleration','r_dot'), rdot_lpv, t);
[ya_fx_og, ~]  = lsim(cl_fixed_og('body acceleration','r_dot'), rdot_lpv, t);
[ya_lv_og, ~]  = lsim(cl_lpv_og('body acceleration','r_dot'),   rdot_lpv, t);
[ys_ol_og, ~]  = lsim(plant_og('rattle space','r_dot'), rdot_lpv, t);
[ys_fx_og, ~]  = lsim(cl_fixed_og('rattle space','r_dot'), rdot_lpv, t);
[ys_lv_og, ~]  = lsim(cl_lpv_og('rattle space','r_dot'),   rdot_lpv, t);
 
fprintf('  OL:        accel=%.4f m/s^2  rattle=%.4f cm\n', rms(ya_ol_og), rms(ys_ol_og)*100);
fprintf('  Fixed-Hinf:accel=%.4f m/s^2  rattle=%.4f cm\n', rms(ya_fx_og), rms(ys_fx_og)*100);
fprintf('  LPV-Hinf:  accel=%.4f m/s^2  rattle=%.4f cm\n', rms(ya_lv_og), rms(ys_lv_og)*100);
 
n_plot_lpv = min(length(t), round(10/dt_lpv));
tp_lpv = t(1:n_plot_lpv);
 
figure(12);
subplot(2,1,1);
plot(tp_lpv, ya_ol_og(1:n_plot_lpv),  'b--','LineWidth',0.8); hold on;
plot(tp_lpv, ya_fx_og(1:n_plot_lpv),  'r',  'LineWidth',1.2);
plot(tp_lpv, ya_lv_og(1:n_plot_lpv),  'k',  'LineWidth',1.2);
title(sprintf('Body Acceleration — ms=%d kg (off-grid, loaded vehicle)',ms_offgrid));
ylabel('m/s^2');
legend(sprintf('OL (%.4f m/s^2)',      rms(ya_ol_og)),...
       sprintf('Fixed H-inf (%.4f)',   rms(ya_fx_og)),...
       sprintf('LPV-H-inf (%.4f)',     rms(ya_lv_og)),'Location','NorthEast');
grid on;
 
subplot(2,1,2);
plot(tp_lpv, ys_ol_og(1:n_plot_lpv)*100,  'b--','LineWidth',0.8); hold on;
plot(tp_lpv, ys_fx_og(1:n_plot_lpv)*100,  'r',  'LineWidth',1.2);
plot(tp_lpv, ys_lv_og(1:n_plot_lpv)*100,  'k',  'LineWidth',1.2);
title('Rattle Space');
ylabel('cm'); xlabel('Time (s)');
legend(sprintf('OL (%.4f cm)',         rms(ys_ol_og)*100),...
       sprintf('Fixed H-inf (%.4f)',   rms(ys_fx_og)*100),...
       sprintf('LPV-H-inf (%.4f)',     rms(ys_lv_og)*100),'Location','NorthEast');
grid on;
sgtitle(sprintf('LPV vs Fixed H-inf at Off-Grid Mass ms=%d kg (V=25 m/s)',ms_offgrid));
 
set(findall(0,'type','figure'),'WindowStyle','docked');
 
%% =========================================================================
%% LOCAL HELPER FUNCTION — must be at end of script file
%% =========================================================================
% MATLAB requires local functions to be at the very end of a script.
 
function K_out = interpolate_controller(ms_val, ms_grid, K_cell)
% INTERPOLATE_CONTROLLER  Linearly interpolate H-inf controller state-space
%   matrices between grid points on the mass scheduling axis.
%
%   K_out = interpolate_controller(ms_val, ms_grid, K_cell)
%
%   ms_val  — current sprung mass [kg]
%   ms_grid — vector of design masses [kg]
%   K_cell  — cell array of ss objects at each grid point
 
    % Clamp to grid bounds (do not extrapolate)
    ms_val = max(ms_grid(1), min(ms_grid(end), ms_val));
 
    % Find surrounding grid points
    idx = find(ms_grid <= ms_val, 1, 'last');
    if idx == length(ms_grid)
        % At or beyond upper bound — use last controller directly
        K_out = K_cell{end};
        return;
    end
 
    % Linear interpolation weight
    alpha = (ms_val - ms_grid(idx)) / (ms_grid(idx+1) - ms_grid(idx));
 
    % Extract state-space matrices from both neighbours
    K1 = ss(K_cell{idx});
    K2 = ss(K_cell{idx+1});
 
    % Interpolate each matrix
    A_K = (1-alpha)*K1.A + alpha*K2.A;
    B_K = (1-alpha)*K1.B + alpha*K2.B;
    C_K = (1-alpha)*K1.C + alpha*K2.C;
    D_K = (1-alpha)*K1.D + alpha*K2.D;
 
    K_out = ss(A_K, B_K, C_K, D_K);
    K_out.InputName  = {'rattle space','body acceleration'};
    K_out.OutputName = {'u'};
end
