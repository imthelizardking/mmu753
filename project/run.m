clf;
clear all; clc;
%% active suspension model used in butsuen for LQR control extended for h-infinity synthesis
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
params = 'br';
if strcmp(params,'bd')
    % Physical parameters (taken from bd example)
    ms = 300;    % kg
    mu = 60;     % kg
    bs = 1000;   % N/m/s
    ks = 16000 ; % N/m
    kt = 190000; % N/m
    bt = 0;      % N/m/s
elseif strcmp(params,'br')
    % Physical parameters (taken from 1989 br)
    ms = 240;    % kg
    mu = 36;     % kg
    bs = 1000;   % N/m/s
    ks = 16000 ; % N/m
    kt = 160000; % N/m
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
%% lqr
% ro1 = 0.4; ro2 = 0.16; ro3 = 0.4; ro4 = 0.16;
% ro1 = 1.0; ro2 = 1.0; ro3 = 1.0; ro4 = 1.0;
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
% quartercar_cl_lqr = ss(A-B*K_lqr, L, C, [0; 0]);
quartercar_cl_lqr.StateName = {'rattle space'; 'body velocity'; 'tire deflection'; 'tire velocity'};
quartercar_cl_lqr.InputName = {'r_dot'};
quartercar_cl_lqr.OutputName = {'body acceleration'; 'rattle space'};
%% gerenate road profile
% generate road and compute open-loop RMS for weight calibration
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
%% hinf(s)
% define weights based on actual signal levels
beta = 0.05;
V = 25;   % must match roadprofile.m u0

Wroad = tf(rms_r * sqrt(2), [1, 0.1]);
Wroad.u = 'd1'; Wroad.y = 'r_dot';

Act = tf(1,1);
Act.InputName = 'u'; Act.OutputName = 'fs';

Wact = 0.05 * tf([1,5],[1,80]);
Wact.u = 'u'; Wact.y = 'e1';

Wd2 = ss(0.01); Wd2.u = 'd2'; Wd2.y = 'Wd2';
Wd3 = ss(0.01); Wd3.u = 'd3'; Wd3.y = 'Wd3';

tighten = 4;  % adjust after seeing new gamma

Wab = tighten*((1-beta)/rms_a) * tf([1/8, 1],[1/0.8, 1]);
Wab.u = 'body acceleration'; Wab.y = 'e2';

Wsd = tighten*(beta/rms_s) * tf([1/8, 1],[1/80, 1]);
Wsd.u = 'rattle space'; Wsd.y = 'e3';

% build interconnection and synthesize
sdmeas = sumblk('y1 = rattle space + Wd2');
abmeas = sumblk('y2 = body acceleration + Wd3');

ICinputs  = {'d1';'d2';'d3';'u'};
ICoutputs = {'e1';'e2';'e3';'y1';'y2'};

qcaric = connect(quartercar(1:2,:), Act, Wroad, Wact, Wab, Wsd, ...
                 Wd2, Wd3, sdmeas, abmeas, ICinputs, ICoutputs);

[K_hinf, ~, gamma] = hinfsyn(qcaric, 2, 1);
fprintf('gamma = %.4f\n', gamma);

K_hinf.InputName  = {'rattle space','body acceleration'};
K_hinf.OutputName = {'u'};

quartercar_cl_hinf = connect(quartercar, Act, K_hinf, ...
    'r_dot', {'body acceleration';'rattle space'});
%% bodes
clf
figure(1);
bodemag(quartercar({'body acceleration';'rattle space'},'r_dot') , 'b--');
hold on
bodemag(quartercar_cl_lqr({'body acceleration';'rattle space'},'r_dot'), 'r');
hold on
bodemag(quartercar_cl_hinf({'body acceleration';'rattle space'},'r_dot') , 'k');
grid minor
legend('Open-loop','LQR','hinf','location','SouthEast')
%% time sim.s
% We simulate the response of all three systems to the same road profile.
[y_ol, ~]   = lsim(quartercar('body acceleration','r_dot'), rdot_iso, t);
[y_lqr, ~]  = lsim(quartercar_cl_lqr('body acceleration','r_dot'), rdot_iso, t);
[y_hinf, ~] = lsim(quartercar_cl_hinf('body acceleration','r_dot'), rdot_iso, t);

% Also simulate Rattle Space for a "Bottoming Out" check
[y_ol_sd, ~]   = lsim(quartercar('rattle space','r_dot'), rdot_iso, t);
[y_lqr_sd, ~]  = lsim(quartercar_cl_lqr('rattle space','r_dot'), rdot_iso, t);
[y_hinf_sd, ~] = lsim(quartercar_cl_hinf('rattle space','r_dot'), rdot_iso, t);

%% 3. Performance Metrics
disp ('gamma=');
disp(gamma);
% Lower RMS = Better Comfort (ISO 2631 standard)
rms_ol = rms(y_ol);
rms_lqr = rms(y_lqr);
rms_hinf = rms(y_hinf);

fprintf('\n--- Performance Results (RMS Acceleration) ---\n');
fprintf('Open-Loop: %.4f m/s^2\n', rms_ol);
fprintf('LQR:       %.4f m/s^2\n', rms_lqr);
fprintf('H-Infinity: %.4f m/s^2\n', rms_hinf);

rms_ol_sd = rms(y_ol_sd);
rms_lqr_sd = rms(y_lqr_sd);
rms_hinf_sd = rms(y_hinf_sd);

fprintf('\n--- Performance Results (RMS Rattle Space) ---\n');
fprintf('Open-Loop: %.4f m/s^2\n', rms_ol_sd);
fprintf('LQR:       %.4f m/s^2\n', rms_lqr_sd);
fprintf('H-Infinity: %.4f m/s^2\n', rms_hinf_sd);

% Visualization
figure(2);

% Top Plot: Road Input
subplot(3,1,1);
plot(t, hsum*100, 'Color', [0.5 0.5 0.5]);
title('ISO 8608 Road Profile (Input)');
ylabel('Displacement [cm]'); grid on;

% Middle Plot: Comfort (Acceleration)
subplot(3,1,2);
plot(t, y_ol, 'b--', 'LineWidth', 0.5); hold on;
plot(t, y_lqr, 'r', 'LineWidth', 1);
plot(t, y_hinf, 'k', 'LineWidth', 1);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); legend('OL', 'LQR', 'Hinf'); grid on;

% Bottom Plot: Suspension Travel (Rattle Space)
subplot(3,1,3);
plot(t, y_ol_sd*100, 'b--', 'LineWidth', 0.5); hold on;
plot(t, y_lqr_sd*100, 'r', 'LineWidth', 1);
plot(t, y_hinf_sd*100, 'k', 'LineWidth', 1);
title('Rattle Space (Suspension Travel)');
ylabel('Travel [cm]'); xlabel('Time [s]'); grid on;

% plot body acc alone
figure(3);
plot(t, y_ol, 'b', 'LineWidth', 1); hold on;
plot(t, y_hinf, 'k', 'LineWidth', 1);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); legend('OL', 'Hinf'); grid on;

% plot rattlespace alone
figure(4);
plot(t, y_ol, 'b', 'LineWidth', 1); hold on;
plot(t, y_hinf, 'k', 'LineWidth', 1);
title('Rattle space');
ylabel('m'); legend('OL', 'Hinf'); grid on;