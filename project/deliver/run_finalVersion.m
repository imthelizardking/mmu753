clear all; clc;
%% active suspension model used in butsuen
% for LQR control extended for h-infinity synthesis
% states:
%   x1 = zs-zu (rattle space)
%   x2 = zs_dot (body velocity)
%   x3 = zu-zr (tire deflection)
%   x4 = zu_dot (tire velocity)
% inputs:
%   u: actuator force
%   r_dot: road velocity
% outputs:
%   y1 = zs_dot_dot (body acceleration)
%   y2 = zs-zu (=x1) (rattle space)
%% init.s
% close all;
set(groot, 'DefaultFigureWindowStyle', 'docked');
%% generate road profile
road_type = 'Type D';
switch road_type
    % ── ISO 8608 Standard Classes ──────────────────────────────────
        case 'Type A',  u0 = 60.00;   % 3-sigma no-lift-off (capped)
        case 'Type B',  u0 = 60.00;   % 3-sigma no-lift-off (capped)
        case 'Type C',  u0 = 33.36;
        case 'Type D',  u0 = 9.00;
        case 'Type E',  u0 = 4.10;
        case 'Type F',  u0 = 2.97;
        case 'Type G',  u0 = 1.46;
        case 'Type H',  u0 = 0.57;
    otherwise
        error('Unknown road type: "%s"', road_type);
end
[road_sig, t, hsum] = roadprofile_fun(road_type, u0);
rms_road = rms(hsum);
r_iso = hsum(1:length(t));          % road displacement [m]
dt     = t(2) - t(1);
rdot_raw = [diff(hsum)./dt, 0];
rdot_iso = rdot_raw;

%% define parameters
params = 'nom';
if strcmp(params,'bd')
    ms = 300; mu = 60; bs = 1000; ks = 16000; kt = 190000; bt = 0;
elseif strcmp(params,'nom')
    ms = 240; mu = 36; bs = 1000; ks = 16000; kt = 160000; bt = 0;
elseif strcmp(params,'suv')
    ms = 600; mu = 60; bs = 3000; ks = 20000; kt = 180000; bt = 0;
elseif strcmp(params,'truck')
    ms = 4000; mu = 200; bs = 8000; ks = 150000; kt = 800000; bt = 0;
elseif strcmp(params,'sunlusoy')
    ms = 250; mu = 35; bs = 800; ks = 15000; kt = 120000; bt = 0;
end

% system dynamics
A = [0, 1, 0, -1;...
    -ks/ms, -bs/ms, 0, bs/ms;...
    0, 0, 0, 1;...
    ks/mu, bs/mu, -kt/mu, -(bs+bt)/mu];
B = [0; 1/ms; 0; -1/mu];
L = [0; 0; -1; 0];
B_x = [B,L];
C = [A(2,:); 1, 0, 0, 0; 0, 0, 1, 0];
D = [1/ms, 0; 0, 0; 0, 0];

quartercar = ss(A, B_x, C, D);
quartercar.StateName = {'rattle space'; 'body velocity'; 'tire deflection'; 'tire velocity'};
quartercar.InputName = {'fs','r_dot'};
quartercar.OutputName = {'body acceleration'; 'rattle space'; 'tire deflection'};
quartercar_openloop = quartercar(:,'r_dot');

%% time simulation for open-loop
[y_ol_a,~] = lsim(quartercar('body acceleration','r_dot'), rdot_iso, t);
[y_ol_s,~] = lsim(quartercar('rattle space','r_dot'),      rdot_iso, t);
rms_a = rms(y_ol_a);
rms_s = rms(y_ol_s);
rms_r = rms(rdot_iso);

%% lqr design
ro1 = 0.4; ro2 = 0.16; ro3 = 0.4; ro4 = 0.16;
Q = [ks^2/ms^2+ro1, bs*ks/ms^2,    0,   -bs*ks/ms^2;...
    bs*ks/ms^2,    bs^2/ms^2+ro2,  0,   -bs^2/ms^2;...
    0,             0,              ro3, 0;...
    -bs*ks/ms^2,   -bs^2/ms^2,     0,   bs^2/ms^2+ro4];
N = [-ks/ms^2; -bs/ms^2; 0; bs/ms^2];
R = 1/ms^2;
K_lqr = lqr(A, B, Q, R, N);
A_cl = A-B*K_lqr;
C_cl = [A_cl(2,:); 1, 0, 0, 0; 0, 0, 1, 0];
quartercar_cl_lqr = ss(A-B*K_lqr,L,C_cl,D(:,2));
quartercar_cl_lqr.StateName = {'rattle space'; 'body velocity'; 'tire deflection'; 'tire velocity'};
quartercar_cl_lqr.InputName = {'r_dot'};
quartercar_cl_lqr.OutputName = {'body acceleration'; 'rattle space'; 'tire deflection'};

% temp lqr analysis
A_cl = A - B*K_lqr;
C_u  = -K_lqr;
D_u  = 0;            
sys_lqr_u = ss(A_cl, L, C_u, D_u);
sys_lqr_u.InputName  = 'r_dot';
sys_lqr_u.OutputName = 'u';
[u_lqr, ~] = lsim(sys_lqr_u, rdot_iso, t);

fprintf('--- LQR control force ---\n');
fprintf('  RMS:  %.2f N\n', rms(u_lqr));
fprintf('  Peak: %.2f N\n', max(abs(u_lqr)));

%% hinf design
[quartercar_cl_hinf, K, gamma, qcar,...
    info, rdot_iso, r_iso] = ...
    design_hinf_for_road('Type A',...
    1,1,3, 0.05); % synthsize A

%% open-loop vs lqr bode
figure(1); clf;
opts = bodeoptions;
opts.FreqUnits = 'Hz'; opts.MagScale  = 'log';
opts.MagUnits  = 'abs'; opts.YLim = [10^-6, 10^2];
bodemag(quartercar_openloop, 'b--', opts); hold on
bodemag(quartercar_cl_lqr,'r', opts); 
grid minor
legend('Open-loop','LQR','location','SouthEast')
title('Open-loop vs LQR Bode');
xlim([0.01, 1000])

%% open-loop vs h-infinity bode
figure(2); clf;
opts = bodeoptions;
opts.FreqUnits = 'Hz'; opts.MagScale  = 'log';
opts.MagUnits  = 'abs'; opts.YLim = [10^-6, 10^3];
bodemag(qcar(['ab';'sd';'td'], 'r'), 'b--', opts); hold on
bodemag(quartercar_cl_hinf(['ab';'sd';'td'], 'r'), 'k', opts);
grid minor
legend('Open-loop','H-infinity','location','SouthEast')
title('Open-loop vs H-infiinty Bode');
xlim([0.01, 1000])

%% time simulations
[y_ab_ol, ~]      = lsim(quartercar('body acceleration','r_dot'), rdot_iso, t);
[y_ab_lqr, ~]     = lsim(quartercar_cl_lqr('body acceleration','r_dot'), rdot_iso, t);
[y_ab_hinf, ~]    = lsim(quartercar_cl_hinf('ab','r'), r_iso, t);
[y_td_ol, ~]   = lsim(quartercar('tire deflection','r_dot'), rdot_iso, t);
[y_td_lqr, ~]  = lsim(quartercar_cl_lqr('tire deflection','r_dot'), rdot_iso, t);
[y_td_hinf, ~] = lsim(quartercar_cl_hinf('td','r'), r_iso, t);
[y_rs_ol, ~]   = lsim(quartercar('rattle space','r_dot'), rdot_iso, t);
[y_rs_lqr, ~]  = lsim(quartercar_cl_lqr('rattle space','r_dot'), rdot_iso, t);
[y_rs_hinf, ~] = lsim(quartercar_cl_hinf('sd','r'), r_iso, t);

figure(3); clf;
subplot(3,1,1);
plot(t, y_ab_ol,  'b--','LineWidth',0.8); hold on;
plot(t, y_ab_lqr, 'r',  'LineWidth',1.0);
plot(t, y_ab_hinf,'k',  'LineWidth',1.0);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_ab_ol)), ...
       sprintf('LQR (%.4f)',  rms(y_ab_lqr)), ...
       sprintf('H-inf (%.4f)',rms(y_ab_hinf)));
xlim([10, 15]);
subplot(3,1,2);
plot(t, y_td_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t, y_td_lqr*100, 'r',  'LineWidth',1.0);
plot(t, y_td_hinf*100,'k',  'LineWidth',1.0);
title('Tire Deflection');
ylabel('cm'); xlabel('Time (s)'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_rs_ol)*100), ...
    sprintf('LQR (%.4f)',  rms(y_rs_lqr)*100), ...
    sprintf('H-inf (%.4f)',rms(y_rs_hinf)*100));
xlim([10, 15]);
subplot(3,1,3);
plot(t, y_rs_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t, y_rs_lqr*100, 'r',  'LineWidth',1.0);
plot(t, y_rs_hinf*100,'k',  'LineWidth',1.0);
title('Suspension Deflection');
ylabel('cm'); xlabel('Time (s)'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_rs_ol)*100), ...
    sprintf('LQR (%.4f)',  rms(y_rs_lqr)*100), ...
    sprintf('H-inf (%.4f)',rms(y_rs_hinf)*100));
xlim([10, 15]);

% 4-8 hz analysis
% Define frequency bounds in Hz
f_lower = 4; 
f_higher = 8;

% --- 1. Compute 4-8 Hz Band-Limited RMS using FFT Power Spectral Density ---
Fs = 1 / (t(2) - t(1)); % Sampling frequency
N_fft = length(t);
freq = (0:N_fft-1) * (Fs / N_fft); % Frequency vector

% Find indices corresponding to the 4-8 Hz band
band_idx = (freq >= f_lower & freq <= f_higher);

% Calculate FFTs of the body acceleration signals
fft_ol   = fft(y_ab_ol)   / N_fft;
fft_hinf = fft(y_ab_hinf) / N_fft;

% Extract single-sided power spectrum scaled back to RMS
% (Multiply by 2 to account for negative frequencies skipped in single-sided)
rms_4_8_ol   = sqrt(sum(2 * abs(fft_ol(band_idx)).^2));
rms_4_8_hinf = sqrt(sum(2 * abs(fft_hinf(band_idx)).^2));

% Print the results nicely to the Command Window
fprintf('\n==================================================\n');
fprintf('     4 - 8 Hz BANDS BODY ACCELERATION PERFORMANCE   \n');
fprintf('==================================================\n');
fprintf('  Open-Loop (Passive) RMS (4-8 Hz):  %.4f m/s^2\n', rms_4_8_ol);
fprintf('  H-Infinity Active   RMS (4-8 Hz):  %.4f m/s^2\n', rms_4_8_hinf);
fprintf('  Vibration Reduction Profile:       %.1f%%\n', ...
        (rms_4_8_ol - rms_4_8_hinf) / rms_4_8_ol * 100);
fprintf('==================================================\n');

% --- 2. Zoomed-In Bode Plot (4-8 Hz) for Body Acceleration ---
figure(501); clf;
opts_zoom = bodeoptions;
opts_zoom.FreqUnits = 'Hz'; 
opts_zoom.MagScale  = 'linear'; % Linear scale is much easier to evaluate absolute differences
opts_zoom.MagUnits  = 'abs'; 

% Isolate just the 'body acceleration' channels ('ab' for H-inf, 1st output for Open-loop)
sys_ol_acc   = qcar('ab', 'r'); 
sys_hinf_acc = quartercar_cl_hinf('ab', 'r');

% Plot the magnitudes
bodemag(sys_ol_acc, 'b--', opts_zoom); hold on;
bodemag(sys_hinf_acc, 'k', opts_zoom);

% Set strict limits on the x-axis to zoom directly into the target zone
xlim([f_lower - 1, f_higher + 2]); % 3 Hz to 10 Hz window to see the boundaries clearly
grid minor;

legend(sprintf('Open-loop (4-8Hz RMS: %.3f)', rms_4_8_ol), ...
       sprintf('H-infinity (4-8Hz RMS: %.3f)', rms_4_8_hinf), ...
       'location', 'NorthEast');
title('Zoomed Body Acceleration Frequency Response (Human Discomfort Zone)');
%% find safe speeds (no tire liftoff)
% For each ISO class, find the highest speed at which the PASSIVE car never
% lifts the tire (peak dynamic tire force < static load). Speed enters only by
% time-scaling the fixed spatial profile, so we seed the RNG to keep the road
% realisation identical while bisecting on speed.
roads_list = {'Type A','Type B','Type C','Type D','Type E','Type F','Type G','Type H'};
seed_road  = 1;       % fixed seed -> reproducible profiles
u_cap      = 60;      % upper speed bound [m/s]

u_safe = zeros(1,numel(roads_list));
fprintf('\n--- Max safe speed (no open-loop tire lift-off) ---\n');
fprintf('  road    | max safe speed [m/s] | note\n');
fprintf('  --------+----------------------+----------------------------\n');
for i = 1:numel(roads_list)
    [u_safe(i), lifted] = max_safe_speed_for_road(roads_list{i}, 'nom', seed_road, u_cap);
    if lifted, note = ''; else, note = sprintf('(no lift-off up to %g; capped)', u_cap); end
    fprintf('  %-7s | %18.2f   | %s\n', roads_list{i}, u_safe(i), note);
end

% ===== PER-ROAD AUTO-GAMMA H-INFINITY SYNTHESIS ==========================
% Uses synth_hinf_auto (the knob bisection, now a function) so you never set
% knob by hand: it auto-tunes knob until gamma lands near target_gamma.
beta_syn     = 0.5;    % comfort/handling weight for these controllers
target_gamma = 0.95;

fprintf('\n--- Per-road auto-gamma synthesis (beta = %.2f) ---\n', beta_syn);
fprintf('  road    | u_safe[m/s] |  knob   | gamma\n');
fprintf('  --------+-------------+---------+--------\n');
K_road = cell(1,numel(roads_list));
g_road = zeros(1,numel(roads_list));
knob_road = zeros(1,numel(roads_list));
for i = 1:numel(roads_list)
    [K_road{i}, g_road(i), knob_road(i)] = hinf_bisection(roads_list{i}, beta_syn, target_gamma);
    fprintf('  %-7s | %9.2f   | %7.3f | %6.4f\n', ...
            roads_list{i}, u_safe(i), knob_road(i), g_road(i));
end
%% ===== PER-ROAD BETA SWEEP: 5 controllers, responses + RMS ==============
% For each road class A..H: synthesise 5 H-infinity controllers with beta
% from 0.05 to 0.95 (knob auto-tuned by synth_hinf_auto), simulate each on
% that road, plot body acceleration / tire deflection / suspension deflection
% with RMS in the legend, and print an RMS table to the command window.
% NOTE: uses get_speed_for_road (should hold your 3-sigma speeds) so synthesis
% and simulation happen at the same per-road speed. ~5 controllers x 8 roads
% of auto-gamma synthesis -> this section can take a few minutes.
set(groot,'DefaultFigureWindowStyle','normal');   % undocked for this section
roads_sweep  = {'Type A','Type B','Type C','Type D','Type E','Type F','Type G','Type H'};
beta_vec     = linspace(0.05, 0.95, 5);
target_gamma = 0.95;
seed_sim     = 1;

for ir = 1:numel(roads_sweep)
    rt  = roads_sweep{ir};
    u0r = get_speed_for_road(rt);

    % --- road profile for this class -------------------------------------
    rng(seed_sim);                                  % reproducible realisation
    [~, tr, hr] = roadprofile_fun(rt, u0r);
    r_iso = hr(1:numel(tr));                         % road displacement
    dtr   = tr(2)-tr(1);
    rdot  = [diff(hr)./dtr, 0];                      % road velocity

    % --- open-loop (passive) reference -----------------------------------
    ab_ol = lsim(quartercar('body acceleration','r_dot'), rdot, tr);
    td_ol = lsim(quartercar('tire deflection','r_dot'),   rdot, tr);
    sd_ol = lsim(quartercar('rattle space','r_dot'),      rdot, tr);

    % --- synthesise the 5 controllers and simulate -----------------------
    nb = numel(beta_vec);
    AB = zeros(numel(tr),nb); TD = AB; SD = AB;
    rms_ab=zeros(1,nb); rms_td=rms_ab; rms_sd=rms_ab; g_b=rms_ab;
    for ib = 1:nb
        fprintf('  %s : synthesising beta=%.2f (%d/%d)...\n', rt, beta_vec(ib), ib, nb);
        [~, g_b(ib), ~, cl] = hinf_bisection(rt, beta_vec(ib), target_gamma);
        AB(:,ib) = lsim(cl('ab','r'), r_iso, tr);
        TD(:,ib) = lsim(cl('td','r'), r_iso, tr);
        SD(:,ib) = lsim(cl('sd','r'), r_iso, tr);
        rms_ab(ib)=rms(AB(:,ib)); rms_td(ib)=rms(TD(:,ib)); rms_sd(ib)=rms(SD(:,ib));
    end

    % --- figure: 3 subplots, RMS in legend -------------------------------
    tw0 = 0.4*tr(end);  tw1 = min(tw0+5, tr(end));   % display window (edit if you like)
    cols = lines(nb);
    figure(100+ir); clf;
    sgtitle(sprintf('Road %s — H_\\infty \\beta-sweep (u_0 = %.2f m/s): comfort \\rightarrow handling', rt(end), u0r));
    subplot(3,1,1); hold on; grid on;
    plot(tr, ab_ol, '--','Color',[.6 .6 .6],'DisplayName',sprintf('OL (%.3f)',rms(ab_ol)));
    for ib=1:nb
        plot(tr, AB(:,ib),'Color',cols(ib,:),'DisplayName',sprintf('\\beta=%.2f (%.3f)',beta_vec(ib),rms_ab(ib)));
    end
    ylabel('a_b [m/s^2]'); legend('Location','eastoutside'); xlim([tw0 tw1]);
    title(sprintf('%s  (u_0 = %.2f m/s) — Body Acceleration', rt, u0r));

    subplot(3,1,2); hold on; grid on;
    plot(tr, td_ol*100,'--','Color',[.6 .6 .6],'DisplayName',sprintf('OL (%.3f)',rms(td_ol)*100));
    for ib=1:nb
        plot(tr, TD(:,ib)*100,'Color',cols(ib,:),'DisplayName',sprintf('\\beta=%.2f (%.3f)',beta_vec(ib),rms_td(ib)*100));
    end
    ylabel('td [cm]'); legend('Location','eastoutside'); xlim([tw0 tw1]);
    title('Tire Deflection (Road Holding)');

    subplot(3,1,3); hold on; grid on;
    plot(tr, sd_ol*100,'--','Color',[.6 .6 .6],'DisplayName',sprintf('OL (%.3f)',rms(sd_ol)*100));
    for ib=1:nb
        plot(tr, SD(:,ib)*100,'Color',cols(ib,:),'DisplayName',sprintf('\\beta=%.2f (%.3f)',beta_vec(ib),rms_sd(ib)*100));
    end
    ylabel('sd [cm]'); xlabel('time [s]'); legend('Location','eastoutside'); xlim([tw0 tw1]);
    title('Suspension Deflection (Rattle Space)');

    % --- command-window RMS table ----------------------------------------
    fprintf('\n==== %s  (u0 = %.2f m/s) — RMS over beta ====\n', rt, u0r);
    fprintf('  beta  | gamma  | body acc [m/s^2] | tire defl [cm] | susp defl [cm]\n');
    fprintf('  ------+--------+------------------+----------------+---------------\n');
    fprintf('   OL   |   --   | %16.4f | %14.4f | %13.4f\n', rms(ab_ol), rms(td_ol)*100, rms(sd_ol)*100);
    for ib=1:nb
        fprintf('  %4.2f  | %6.4f | %16.4f | %14.4f | %13.4f\n', ...
                beta_vec(ib), g_b(ib), rms_ab(ib), rms_td(ib)*100, rms_sd(ib)*100);
    end
    fprintf('\n');
    % --- auto-save the figure -------------------------------------------------
    if ~exist('figs','dir'), mkdir('figs'); end          % output folder
    fname = sprintf('betasweep_Road_%s', rt(end));        % e.g. betasweep_Road_A
    fig = figure(100+ir);
    set(fig,'Units','pixels','Position',[100 100 1100 850]);   % consistent size
    exportgraphics(fig, fullfile('figs',[fname '.png']), 'Resolution', 200);
    fprintf('  saved: %s.png\n', fname);
end
% ===== end of per-road beta sweep =====

%% = CROSS-TEST: controller-designed-on-X evaluated on road-Y =========
% For each beta in {0.05, 0.5, 0.95}:
%   - synthesise 8 controllers, one designed on each road A..H
%   - for each TEST road A..H, overlay all 8 controllers' responses (3 subplots)
%   => 3 x 8 = 24 figures. Matched controller (design==test) is drawn thick.
% Reuses synth_hinf_auto, get_speed_for_road, roadprofile_fun, quartercar.
% The closed loop (road->outputs) is a fixed LTI system, so cross-testing only
% means feeding the test road's profile into each controller's closed loop.

roads  = {'Type A','Type B','Type C','Type D','Type E','Type F','Type G','Type H'};
letters= cellfun(@(s) s(end), roads);                 % 'A'..'H'
betas  = [0.05 0.50 0.95];
btag   = {'b005','b050','b095'};
nR     = numel(roads);

% --- precompute each TEST road's profile (input signals + OL) -------------
TR = struct('t',{},'r',{},'rdot',{},'u0',{},'ab_ol',{},'td_ol',{},'sd_ol',{});
for j = 1:nR
    u0j = get_speed_for_road(roads{j});
    rng(1);  [~, tj, hj] = roadprofile_fun(roads{j}, u0j);
    rj   = hj(1:numel(tj));
    rdj  = [diff(hj)./(tj(2)-tj(1)), 0];
    TR(j).t=tj; TR(j).r=rj; TR(j).rdot=rdj; TR(j).u0=u0j;
    TR(j).ab_ol = lsim(quartercar('body acceleration','r_dot'), rdj, tj);
    TR(j).td_ol = lsim(quartercar('tire deflection','r_dot'),   rdj, tj);
    TR(j).sd_ol = lsim(quartercar('rattle space','r_dot'),      rdj, tj);
end

% --- output folder + undock figures so screenshots size correctly --------
if ~exist('figs_cross','dir'), mkdir('figs_cross'); end
prevStyle = get(groot,'DefaultFigureWindowStyle');
set(groot,'DefaultFigureWindowStyle','normal');

cmap = turbo(nR);                                     % 8 distinct controller colours

% storage: RMS(metric).(beta){design,test}
RMSab = zeros(nR,nR,numel(betas));
RMStd = zeros(nR,nR,numel(betas));
RMSsd = zeros(nR,nR,numel(betas));

for ib = 1:numel(betas)
    beta = betas(ib);

    % ---- synthesise the 8 controllers for this beta ---------------------
    CL = cell(1,nR);
    for i = 1:nR
        fprintf('beta=%.2f : synthesising K designed on %s (%d/%d)...\n', beta, roads{i}, i, nR);
        [~,~,~,cl] = hinf_bisection(roads{i}, beta, 0.95);
        CL{i} = cl;
    end

    % ---- for each TEST road, overlay all 8 controllers ------------------
    for j = 1:nR
        t = TR(j).t;  r = TR(j).r;
        tw0 = 0.4*t(end);  tw1 = min(tw0+5, t(end));

        fig = figure(300 + (ib-1)*nR + j); clf;

        % subplot handles
        s1=subplot(3,1,1); hold on; grid on;
        s2=subplot(3,1,2); hold on; grid on;
        s3=subplot(3,1,3); hold on; grid on;

        % open-loop reference (grey dashed)
        plot(s1, t, TR(j).ab_ol,       '--','Color',[.6 .6 .6],'DisplayName',sprintf('OL (%.3f)',rms(TR(j).ab_ol)));
        plot(s2, t, TR(j).td_ol*100,   '--','Color',[.6 .6 .6],'DisplayName',sprintf('OL (%.3f)',rms(TR(j).td_ol)*100));
        plot(s3, t, TR(j).sd_ol*100,   '--','Color',[.6 .6 .6],'DisplayName',sprintf('OL (%.3f)',rms(TR(j).sd_ol)*100));

        for i = 1:nR
            ab = lsim(CL{i}('ab','r'), r, t);
            td = lsim(CL{i}('td','r'), r, t);
            sd = lsim(CL{i}('sd','r'), r, t);
            RMSab(i,j,ib)=rms(ab); RMStd(i,j,ib)=rms(td); RMSsd(i,j,ib)=rms(sd);

            matched = (i==j);
            lw = 2.0*matched + 1.0*~matched;            % thick if design==test
            tagm = ''; if matched, tagm='*'; end
            nm_ab = sprintf('K_{%s}%s (%.3f)', letters(i), tagm, RMSab(i,j,ib));
            nm_td = sprintf('K_{%s}%s (%.3f)', letters(i), tagm, RMStd(i,j,ib)*100);
            nm_sd = sprintf('K_{%s}%s (%.3f)', letters(i), tagm, RMSsd(i,j,ib)*100);
            plot(s1, t, ab,     'Color',cmap(i,:),'LineWidth',lw,'DisplayName',nm_ab);
            plot(s2, t, td*100, 'Color',cmap(i,:),'LineWidth',lw,'DisplayName',nm_td);
            plot(s3, t, sd*100, 'Color',cmap(i,:),'LineWidth',lw,'DisplayName',nm_sd);
        end

        ylabel(s1,'a_b [m/s^2]'); ylabel(s2,'td [cm]'); ylabel(s3,'sd [cm]'); xlabel(s3,'time [s]');
        title(s1,'Body Acceleration'); title(s2,'Tire Deflection'); title(s3,'Suspension Deflection');
        xlim(s1,[tw0 tw1]); xlim(s2,[tw0 tw1]); xlim(s3,[tw0 tw1]);
        legend(s1,'Location','eastoutside'); legend(s2,'Location','eastoutside'); legend(s3,'Location','eastoutside');
        sgtitle(sprintf('Cross-test (\\beta=%.2f): controllers K_{A..H} evaluated on Road %s  (u_0=%.2f m/s)', ...
                beta, letters(j), TR(j).u0));

        % ---- save screenshot --------------------------------------------
        set(fig,'Units','pixels','Position',[100 100 1300 950]); drawnow;
        fname = sprintf('crosstest_%s_Road%s', btag{ib}, letters(j));
        exportgraphics(fig, fullfile('figs_cross',[fname '.png']), 'Resolution', 200);
        fprintf('  saved: %s.png\n', fname);
    end

    % ---- command-window cross matrices (rows=design, cols=test) ---------
    print_cross(sprintf('beta=%.2f  BODY-ACC RMS [m/s^2]',beta), RMSab(:,:,ib),   letters, 1);
    print_cross(sprintf('beta=%.2f  TIRE-DEFL RMS [cm]',  beta), RMStd(:,:,ib)*100,letters, 100/100);
    print_cross(sprintf('beta=%.2f  SUSP-DEFL RMS [cm]', beta), RMSsd(:,:,ib)*100, letters, 1);
end

set(groot,'DefaultFigureWindowStyle',prevStyle);      % restore docking pref
% ===== end of cross-test section =====

%% helpers
function print_cross(ttl, M, letters, ~)
    nR = numel(letters);
    fprintf('\n=== %s  (rows = design road, cols = test road) ===\n', ttl);
    fprintf('  design\\test |');
    for j=1:nR, fprintf('%8s', letters(j)); end
    fprintf('\n  ------------+');
    for j=1:nR, fprintf('--------'); end
    fprintf('\n');
    for i=1:nR
        fprintf('     K_%s     |', letters(i));
        for j=1:nR
            if i==j, fprintf('%7.3f*', M(i,j)); else, fprintf('%8.3f', M(i,j)); end
        end
        fprintf('\n');
    end
    fprintf('  (* = matched: controller tested on its own design road)\n');
end

function [u_safe, lifted] = max_safe_speed_for_road(road_type, params, seed, u_cap)
% Highest speed at which the passive car does NOT lift the tire.
    if nargin<2||isempty(params), params='nom'; end
    if nargin<3||isempty(seed),   seed=1;       end
    if nargin<4||isempty(u_cap),  u_cap=60;     end

    [ms,mu,bs,ks,kt,bt] = get_vehicle_params(params);
    g = 9.81;  F_static = (ms+mu)*g;
    Acar = [0 1 0 -1; -ks/ms -bs/ms 0 bs/ms; 0 0 0 1; ks/mu bs/mu -kt/mu -(bs+bt)/mu];
    L = [0;0;-1;0];  Ctd = [0 0 1 0];
    Gtd = ss(Acar, L, Ctd, 0);          % road velocity -> tire deflection (passive)

    % no lift-off even at the cap -> return cap
    if ~road_lifts_off(u_cap, road_type, seed, Gtd, kt, F_static)
        u_safe = u_cap;  lifted = false;  return;
    end
    % lift-off already at the lowest speed -> return that
    u_lo = .1;  u_hi = u_cap;
    if road_lifts_off(u_lo, road_type, seed, Gtd, kt, F_static)
        u_safe = u_lo;  lifted = true;  return;
    end
    % bisection: u_lo is safe, u_hi lifts
    for it = 1:22
        u_mid = (u_lo+u_hi)/2;
        if road_lifts_off(u_mid, road_type, seed, Gtd, kt, F_static)
            u_hi = u_mid;
        else
            u_lo = u_mid;
        end
    end
    u_safe = u_lo;  lifted = true;
end

function tf = road_lifts_off(u0, road_type, seed, Gtd, kt, F_static)
    rng(seed);                                   % same spatial profile every call
    [~, t, hsum] = roadprofile_fun(road_type, u0);
    dt   = t(2)-t(1);
    rdot = [diff(hsum)./dt, 0];                  % road velocity (scales with u0)
    td   = lsim(Gtd, rdot, t);
    % tf   = max(abs(kt*td)) >= F_static;
    sigma_F = std(kt*td);          % std of dynamic tire force
    tf = 3*sigma_F >= F_static;    % 3-sigma road-holding limit
end

function [K, gamma_out, knob, cl_hinf, qcar_open] = hinf_bisection(road_type, beta, target_gamma, params)
% Knob bisection (your old inline loop, now reusable): auto-tunes knob until
% gamma is near target_gamma, returning the controller at that knob.
    if nargin<3||isempty(target_gamma), target_gamma = 0.95; end
    if nargin<4||isempty(params),       params = 'nom';      end
    gamma_tol = 0.05;  max_iters = 50;
    knob_low  = 0.01;  knob_high = 50.0;
    K = []; gamma_out = Inf; knob = 1; cl_hinf = []; qcar_open = [];

    for it = 1:max_iters
        knob_test = (knob_low + knob_high)/2;
        [cl, Kt, gt, qc] = design_hinf_for_road(road_type, 1, 1, knob_test, beta, params);

        if isempty(gt) || isnan(gt) || isinf(gt)
            knob_high = knob_test;  continue;        % infeasible -> soften weights
        end
        K = Kt;  gamma_out = gt;  knob = knob_test;  cl_hinf = cl;  qcar_open = qc;

        if abs(gt - target_gamma) <= gamma_tol, break; end
        if gt > target_gamma, knob_high = knob_test; else, knob_low = knob_test; end
    end
end

function [cl_hinf, K, gamma_out, qcar_open, info, rdot_iso, r_iso] = design_hinf_for_road(road_type, rms_a_ref, rms_rs_ref, knob, beta, params)
    
    if nargin < 6 || isempty(params)
        params = 'nom';
    end

    u0 = get_speed_for_road(road_type); V = u0;
    [~, t, hsum, Gh0] = roadprofile_fun(road_type, u0);
    rms_road = rms(hsum);
    r_iso    = hsum(1:length(t));
    dt       = t(2) - t(1);
    rdot_iso = [diff(hsum)./dt, 0];

    [ms, mu, bs, ks, kt, bt] = get_vehicle_params(params);   

    % --- Displacement-input plant for H-infinity synthesis ---
    A = [ 0 1 0 0; ...
          [-ks -bs ks bs]/ms; ...
          0 0 0 1; ...
          [ks bs -ks-kt -bs]/mu];
    B = [ 0 0; 0 1e3/ms; 0 0; [kt -1e3]/mu];
    
    % addition (tire deflection): td = x3 - r 
    C = [1 0 0 0; 1 0 -1 0; A(2,:); 0 0 1 0];
    D = [0 0; 0 0; B(2,:); -1 0];

    qcar_open = ss(A, B, C, D);
    qcar_open.StateName  = ["body travel (m)";"body vel (m/s)"; ...
                            "wheel travel (m)";"wheel vel (m/s)"];
    qcar_open.InputName  = ["r";"fs"];
    qcar_open.OutputName = ["xb";"sd";"ab";"td"];

    w_bounce = sqrt(ks/ms); % body bounce freq
    w_wheel_hop = sqrt(kt/mu); % wheel-hop freq

    Act = tf(1,[1/60 1]); % actuator with 60 rad/s cutoff, light but not unity
    Act.InputName = "u"; Act.OutputName = "fs";

    w_road_cut = 2 * pi * 2; % cut-off frequency = 2 Hz
    Wroad = tf(sqrt(Gh0 * V) * w_road_cut, [1, w_road_cut]);
    Wroad.u = "d1"; Wroad.y = "r";
    
    Wact = 0.1 * tf([1, w_bounce], [1, 100]);
    Wact.u = "u"; Wact.y = "e1";

    Wd2 = ss(0.01); Wd2.u = "d2"; Wd2.y = "Wd2"; % measurement noise, small
    Wd3 = ss(0.01);  Wd3.u = "d3"; Wd3.y = "Wd3"; % measurement noise, small

    max_travel = 1.0; % for know it's 1 meters, will change
    scale_factor = 1/max_travel;
    Wsd =  scale_factor * tf(1, [1/w_bounce, 1]);
    Wsd.u = "sd"; Wsd.y = "e3";

    % human are sensitive to 4-8 Hz (25.1-50.3 rad)
    w_lower = 25.1; w_higher = 50.3;
    w_center = (w_lower + w_higher)/2;
    wab_bandwidth = w_higher-w_lower;
    Wab = knob * (1-beta) * tf([wab_bandwidth, 0], [1, wab_bandwidth, w_center^2]);
    Wab.u = "ab"; Wab.y = "e2";

    % Road handling is evaluated at the wheel-hop frequency
    % Wtd = knob * beta * tf(1, [1/w_wheel_hop, 1]);
    Wtd = knob * beta * tf([1, 1], [1, w_wheel_hop]);
    Wtd.u = "td"; Wtd.y = "e4";

    % make necessary connections for plant
    sdmeas = sumblk("y1 = sd+Wd2");
    abmeas = sumblk("y2 = ab+Wd3");
    ICinputs  = ["d1";"d2";"d3";"u"];
    ICoutputs = ["e1";"e2";"e3";"e4";"y1";"y2"];

    % create plant
    qcaric = connect(qcar_open(["sd","ab","td"],:), Act, Wroad, Wact, Wab, Wsd, Wtd, ...
                     Wd2, Wd3, sdmeas, abmeas, ICinputs, ICoutputs);

    [K, ~, gamma_out] = hinfsyn(qcaric, 2, 1); % synthesize hinf controller
    K.u = ["sd","ab"]; K.y = "u";
    
    % Reconnect closed loop with ALL outputs including Tire Deflection
    cl_hinf = connect(qcar_open, Act, K, "r", ["xb";"sd";"ab";"td"]);
    cl_hinf = cl_hinf([3,2,1,4], :); % Orders outputs as: ab, sd, xb, td

    info.road_type = road_type;
    info.u0        = u0;
    info.t         = t;
    info.r_iso     = r_iso;
    info.hsum      = hsum;
    info.rms_road  = rms_road;
    info.rms_a_ref = rms_a_ref;
    info.rms_rs_ref= rms_rs_ref;
    info.ms = ms; info.mu = mu; info.ks = ks; info.kt = kt;
    info.bs = bs; info.bt = bt;
end

function u0 = get_speed_for_road(road_type)
    switch road_type
        case 'Type A',  u0 = 60.00;   % 3-sigma no-lift-off (capped)
        case 'Type B',  u0 = 60.00;   % 3-sigma no-lift-off (capped)
        case 'Type C',  u0 = 33.36;
        case 'Type D',  u0 = 9.00;
        case 'Type E',  u0 = 4.10;
        case 'Type F',  u0 = 2.97;
        case 'Type G',  u0 = 1.46;
        case 'Type H',  u0 = 0.57;
        otherwise
            error('Unknown road type: "%s"', road_type);
    end
end

function [ms, mu, bs, ks, kt, bt] = get_vehicle_params(params)
    switch params
        case 'bd'
            ms = 300; mu = 60;  bs = 1000; ks = 16000;  kt = 190000; bt = 0;
        case 'nom'
            ms = 240; mu = 36;  bs = 1000; ks = 16000;  kt = 160000; bt = 0;
        case 'suv'
            ms = 600; mu = 60;  bs = 3000; ks = 20000;  kt = 180000; bt = 0;
        case 'truck'
            ms = 4000; mu = 200; bs = 8000; ks = 150000; kt = 800000; bt = 0;
        case 'sunlusoy'
            ms = 250; mu = 35;  bs = 800;  ks = 15000;  kt = 120000; bt = 0;
        otherwise
            error('Unknown params: "%s"', params);
    end
end
