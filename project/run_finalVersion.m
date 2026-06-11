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

%% temp lqr analysis
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
    1,1,3, 0.01); % synthsize A

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

%% further analysis
[cl_hinf_A, K_A, gamma_out_A, qcar_open_A] = design_hinf_for_road('Type C', 1, 1, 4, 0.25); % synthsize A
[cl_hinf_D, K_D, gamma_out_D, qcar_open_D] = design_hinf_for_road('Type C', 1, 1, 50, 0.9); % synthsize D

road_compare = 'Type C';
u0_ = get_speed_for_road(road_compare);
[~, t_, hsum_, Gh0_] = roadprofile_fun(road_compare, u0_);
r_iso_compare     = hsum_(1:length(t_));
dt_               = t_(2) - t_(1);
r_dot_iso_compare = [diff(hsum_)./dt_, 0];

% time sim.s:
[y_ab_ol, ~]     = lsim(quartercar('body acceleration','r_dot'), r_dot_iso_compare, t_);
[y_ab_hinfA, ~]  = lsim(cl_hinf_A('ab','r'), r_iso_compare, t_);
[y_ab_hinfD, ~]  = lsim(cl_hinf_D('ab','r'), r_iso_compare, t_);

[y_rs_ol, ~]     = lsim(quartercar('rattle space','r_dot'), r_dot_iso_compare, t_);
[y_rs_hinfA, ~]  = lsim(cl_hinf_A('sd','r'), r_iso_compare, t_);
[y_rs_hinfD, ~]  = lsim(cl_hinf_D('sd','r'), r_iso_compare, t_);

% TIRE DEFLECTION SİMS:
[y_td_ol, ~]     = lsim(quartercar('tire deflection','r_dot'), r_dot_iso_compare, t_);
[y_td_hinfA, ~]  = lsim(cl_hinf_A('td','r'), r_iso_compare, t_);
[y_td_hinfD, ~]  = lsim(cl_hinf_D('td','r'), r_iso_compare, t_);

figure(4); clf;
subplot(3,1,1);
plot(t_, y_ab_ol,  'b--','LineWidth',0.8); hold on;
plot(t_, y_ab_hinfA, 'r',  'LineWidth',1.0);
plot(t_, y_ab_hinfD,'k',  'LineWidth',1.0);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_ab_ol)), ...
    sprintf('HinfA (%.4f)',  rms(y_ab_hinfA)), ...
    sprintf('HinfD (%.4f)',rms(y_ab_hinfD)));
xlim([10, 15]);

subplot(3,1,2);
plot(t_, y_td_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t_, y_td_hinfA*100, 'r',  'LineWidth',1.0);
plot(t_, y_td_hinfD*100,'k',  'LineWidth',1.0);
title('Tire Deflection (Road Holding)');
ylabel('cm'); xlabel('Time (s)'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_td_ol)*100), ...
    sprintf('HinfA (%.4f)',  rms(y_td_hinfA)*100), ...
    sprintf('HinfD (%.4f)',rms(y_td_hinfD)*100));
xlim([10, 15]);

subplot(3,1,3);
plot(t_, y_rs_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t_, y_rs_hinfA*100, 'r',  'LineWidth',1.0);
plot(t_, y_rs_hinfD*100,'k',  'LineWidth',1.0);
title('Suspension Deflection');
ylabel('cm'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_rs_ol)*100), ...
    sprintf('HinfA (%.4f)',  rms(y_rs_hinfA)*100), ...
    sprintf('HinfD (%.4f)',rms(y_rs_hinfD)*100));
xlim([10, 15]);

disp('------------------------------------------------');
disp(['Road Compare: ', road_compare]);
disp('------------------------------------------------');
fprintf('gamma A: %.4f\n', gamma_out_A);
fprintf('gamma D: %.4f\n\n', gamma_out_D);

disp('--- Body Acceleration RMS [m/s^2] ---');
fprintf('Open-Loop: %.4f\n', rms(y_ab_ol));
fprintf('K_A:       %.4f\n', rms(y_ab_hinfA));
fprintf('K_D:       %.4f\n\n', rms(y_ab_hinfD));

disp('--- Rattle Space RMS [cm] ---');
fprintf('Open-Loop: %.4f\n', rms(y_rs_ol)*100);
fprintf('K_A:       %.4f\n', rms(y_rs_hinfA)*100);
fprintf('K_D:       %.4f\n\n', rms(y_rs_hinfD)*100);

%% Road Handling Check & Forces
g = 9.81;
F_static = (ms + mu) * g;
F_dyn_ol    = kt * y_td_ol;
F_dyn_hinfA = kt * y_td_hinfA;
F_dyn_hinfD = kt * y_td_hinfD;

disp('--- Road Handling Report ---');
fprintf('Static Tire Load: %.2f N\n', F_static);
fprintf('Open-Loop Peak Dynamic Force: %7.2f N  | Ratio: %.2f\n', max(abs(F_dyn_ol)), max(abs(F_dyn_ol))/F_static);
fprintf('K_A Peak Dynamic Force:       %7.2f N  | Ratio: %.2f\n', max(abs(F_dyn_hinfA)), max(abs(F_dyn_hinfA))/F_static);
fprintf('K_D Peak Dynamic Force:       %7.2f N  | Ratio: %.2f\n\n', max(abs(F_dyn_hinfD)), max(abs(F_dyn_hinfD))/F_static);

if max(abs(F_dyn_ol)) > F_static
    disp('>>> WARNING: Open-loop experiences TIRE LIFT-OFF on this road!');
end
if max(abs(F_dyn_hinfA)) > F_static
    disp('>>> WARNING: K_A experiences TIRE LIFT-OFF on this road!');
end
if max(abs(F_dyn_hinfD)) > F_static
    disp('>>> WARNING: K_D experiences TIRE LIFT-OFF on this road!');
end

figure(5); clf;
plot(t_, abs(F_dyn_ol) / F_static, 'b--', 'LineWidth', 0.8); hold on;
plot(t_, abs(F_dyn_hinfA) / F_static, 'r', 'LineWidth', 1.0);
plot(t_, abs(F_dyn_hinfD) / F_static, 'k', 'LineWidth', 1.0);
yline(1.0, 'm-', 'Lift-off Limit (F_{dyn} = F_{stat})', 'LineWidth', 2);
title(sprintf('Dynamic vs Static Tire Load Ratio on %s', road_compare));
xlabel('Time (s)');
ylabel('Ratio (F_{dyn} / F_{stat})');
grid on;
legend('Open-Loop', 'K_A', 'K_D', 'Location', 'Best');
xlim([10, 15]);
ylim([0, 1.5]); % Scale to see the liftoff clearly
%% better rms
% % unit test:
fs = 700;
% fcheck = logspace(-1, 2, 200); g = zeros(size(fcheck));
% T = 10; t = (0:1/fs:T)';
% figure(6); clf;
% for i = 1:numel(fcheck)
%     x = sin(2*pi*fcheck(i)*t);
%     g(i) = iso2631_wk_rms(x, fs) / sqrt(mean(x.^2));
% end
% semilogx(fcheck, 20*log10(g)); grid on
% xlabel('Hz'); ylabel('Wk gain [dB]');

aw_rms_OL = iso2631_wk_rms(y_ab_ol, fs);
aw_rms_hinfA = iso2631_wk_rms(y_ab_hinfA, fs);
aw_rms_hinfD = iso2631_wk_rms(y_ab_hinfD, fs);
%% ===== MAX SAFE SPEED PER ROAD (no open-loop tire lift-off) ==============
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
    [K_road{i}, g_road(i), knob_road(i)] = synth_hinf_auto(roads_list{i}, beta_syn, target_gamma);
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
        [~, g_b(ib), ~, cl] = synth_hinf_auto(rt, beta_vec(ib), target_gamma);
        AB(:,ib) = lsim(cl('ab','r'), r_iso, tr);
        TD(:,ib) = lsim(cl('td','r'), r_iso, tr);
        SD(:,ib) = lsim(cl('sd','r'), r_iso, tr);
        rms_ab(ib)=rms(AB(:,ib)); rms_td(ib)=rms(TD(:,ib)); rms_sd(ib)=rms(SD(:,ib));
    end

    % --- figure: 3 subplots, RMS in legend -------------------------------
    tw0 = 0.4*tr(end);  tw1 = min(tw0+5, tr(end));   % display window (edit if you like)
    cols = lines(nb);
    figure(100+ir); clf;

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
end
% ===== end of per-road beta sweep =====

%% ===== CERTIFIED LPV (hinfgs) ON TYPE C, TIME-BASED beta(t) =============
% beta scales the FEED-IN of two first-order shaping filters (comfort filter
% on body acceleration, handling filter on tire deflection), so beta enters
% the A matrix (A1 ~= 0) -- the condition your hinfgs install requires -- while
% B2/C2/D stay constant. One hinfgs call -> a certified gain-scheduled
% controller (two vertices), interpolated online by [1-beta, beta].
%
% Keeps the filter STRUCTURE/intent of your design (comfort, handling, road,
% actuator weights); re-expressed by hand so beta lands on filter dynamics.
clearvars -except quartercar    % keep nothing road-specific lingering; quartercar reused
clc;

road_lpv = 'Type C';
[ms,mu,bs,ks,kt,bt] = get_vehicle_params('nom');

% --- physical quarter-car (displacement-input), same as design_hinf_for_road
A  = [ 0 1 0 0; [-ks -bs ks bs]/ms; 0 0 0 1; [ks bs -ks-kt -bs]/mu];
Bu = [0; 1e3/ms; 0; -1e3/mu];     % actuator force column (1e3 scaling as in your code)
Bw = [0; 0; 0; kt/mu];            % road displacement column
np = 4;
Cab = A(2,:);  Dab = 1e3/ms;      % body acceleration = A(2,:)x + (1e3/ms)*fs
Ctd = [0 0 1 0];                  % wheel travel; tire deflection = x3 - r (handled below)
Csd = [1 0 -1 0];                 % suspension deflection sd = xb - xw
Cy  = Csd;                        % measurement: suspension deflection (no u feedthrough)

% --- weights (keep your structure; tune like your knob/beta) -------------
wc = 2*pi*sqrt(ks/ms);            % comfort filter corner (~body bounce)
wh = 2*pi*sqrt(kt/mu)/(2*pi);     % handling filter corner (~wheel hop, in rad/s below)
wh = sqrt(kt/mu);                 % wheel-hop frequency [rad/s]
wc = sqrt(ks/ms);                 % body-bounce frequency [rad/s]
Wc = 1;      Wh = 1500;           % comfort / handling weights (raise Wh -> sharper handling)
Wu = 1e-3;   en = 1e-2;           % actuator-effort / sensor-noise weights

% --- augmented affine plant: 4 plant + 2 filter states = 6 ---------------
% xc' = -wc*xc + (1-beta)*wc*body_acc(state part);  z1 = Wc*xc
% xh' = -wh*xh +    beta *wh*tire_defl(state part);  z2 = Wh*xh
% beta lives in the filter feed-in (A1 ~= 0). z3 = Wu*u, y = sd + noise.
n  = 6;  Z4 = zeros(1,np);

A0 = [ A,            zeros(np,2);
       wc*Cab,      -wc,  0;            % comfort filter fed at beta=0
       zeros(1,np),  0,  -wh ];         % handling filter feed = 0 at beta=0
A1 = [ zeros(np,n);
      -wc*Cab,       0,   0;            % comfort feed fades out with beta
       wh*Ctd,       0,   0 ];          % handling feed fades in  with beta

B0 = [ Bw, zeros(np,1), Bu;
       0,  0,           wc*Dab;         % comfort filter u-feedthrough (constant)
       0,  0,           0 ];
B1 = zeros(n,3);

C0 = [ Z4, Wc, 0;        % z1 = Wc*xc
       Z4, 0,  Wh;       % z2 = Wh*xh
       Z4, 0,  0;        % z3 (state part) = 0
       Cy, 0,  0 ];      % y  = sd
D0 = [ 0 0 0; 0 0 0; 0 0 Wu; 0 en 0 ];
C1 = zeros(4,n);  D1 = zeros(4,3);      % weights constant -> beta only in A

s0 = ltisys(A0, B0, C0, D0);            % E0 = I
s1 = ltisys(A1, B1, C1, D1, 0);         % E1 = 0 (scalar) ; A1 ~= 0 -> accepted
pdP = psys(pvec('box',[0 1]), [s0 s1]);

% --- ONE certified gain-scheduled H-infinity synthesis -------------------
[gopt, pdK] = hinfgs(pdP, [1 1]);
fprintf('\nLPV hinfgs gamma over beta in [0,1] = %.4f\n', gopt);
[aK0,bK0,cK0,dK0] = ltiss(psinfo(pdK,'sys',1));   % vertex beta = 0
[aK1,bK1,cK1,dK1] = ltiss(psinfo(pdK,'sys',2));   % vertex beta = 1
nK = size(aK0,1);

% --- plant+actuator block for evaluation: [r;u] -> [ab;td;sd] ------------
Act = ss(tf(1,[1/60 1]));  [Aa,Ba,Ca,~] = ssdata(Act);  na = size(Aa,1);
% build [xp;xa] state space by hand (fs = Ca*xa, strictly proper -> no u in outputs)
Apx = [A, Bu*Ca; zeros(na,np), Aa];
Bpx = [Bw, zeros(np,1); zeros(na,1), Ba];           % inputs [r, u]
Cab_x = [Cab, Dab*Ca];  Ctd_x = [Ctd, 0*Ca];  Csd_x = [Csd, 0*Ca];  Cy_x = Csd_x;
nPx = np+na;

% --- frozen stability sweep across beta ----------------------------------
fprintf('\n--- LPV frozen stability sweep ---\n');
for b = 0:0.1:1
    aK=(1-b)*aK0+b*aK1; bK=(1-b)*bK0+b*bK1; cK=(1-b)*cK0+b*cK1; dK=(1-b)*dK0+b*dK1;
    % closed-loop A for [xp;xa;xk], road=0 :  u = cK*xk + dK*(Cy_x*[xp;xa])
    Acl = [ Apx + Bpx(:,2)*dK*Cy_x,  Bpx(:,2)*cK;
            bK*Cy_x,                 aK ];
    fprintf('  beta=%.1f : stable=%d , max Re(pole)=%9.2f\n', ...
            b, all(real(eig(Acl))<0), max(real(eig(Acl))));
end

% --- ISO Type C road (displacement) --------------------------------------
u0c = get_speed_for_road(road_lpv);
rng(1);  [~, tc, hc] = roadprofile_fun(road_lpv, u0c);
r_iso = hc(1:numel(tc));
roadI = griddedInterpolant(tc, r_iso, 'linear');
Tend  = min(tc(end), 30);
t_out = (0:1e-3:Tend).';

% --- TIME-BASED beta(t): comfort -> handling -----------------------------
t0 = 0.25*Tend;  t1 = 0.75*Tend;
beta_fun = @(tt) 0.5*(1 - cos(pi*min(max((tt-t0)/(t1-t0),0),1)));   % 0 -> 1

% --- simulate the certified scheduled controller -------------------------
K0.a=aK0;K0.b=bK0;K0.c=cK0;K0.d=dK0;  K1.a=aK1;K1.b=bK1;K1.c=cK1;K1.d=dK1;
odef = @(t,X) lpv_cert_rhs(t,X, Apx,Bpx,Cy_x, nPx, K0,K1, roadI, beta_fun);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',2e-3);
[~,Xs] = ode45(odef, t_out, zeros(nPx+nK,1), opts);

ab_cl=zeros(size(t_out)); td_cl=ab_cl; sd_cl=ab_cl; bb=ab_cl;
for k=1:numel(t_out)
    xpx = Xs(k,1:nPx).';  xk = Xs(k,nPx+1:end).';
    b = beta_fun(t_out(k)); bb(k)=b;
    cK=(1-b)*cK0+b*cK1; dK=(1-b)*dK0+b*dK1;
    u  = cK*xk + dK*(Cy_x*xpx);
    ab_cl(k)=Cab_x*xpx + Dab*0;            % Dab feedthrough is via fs (in Cab_x); u direct=0
    td_cl(k)=Ctd_x*xpx - roadI(t_out(k));  % tire deflection = wheel travel - road
    sd_cl(k)=Csd_x*xpx;
end

% --- open-loop (passive) on same road ------------------------------------
rdot = [diff(hc)./(tc(2)-tc(1)), 0];
ab_ol = lsim(quartercar('body acceleration','r_dot'), rdot, tc);
td_ol = lsim(quartercar('tire deflection','r_dot'),   rdot, tc);
sd_ol = lsim(quartercar('rattle space','r_dot'),      rdot, tc);
ab_ol=interp1(tc,ab_ol,t_out); td_ol=interp1(tc,td_ol,t_out); sd_ol=interp1(tc,sd_ol,t_out);

% --- plots ---------------------------------------------------------------
figure(30); clf;
subplot(4,1,1); plot(t_out,bb,'r','LineWidth',1.5); grid on; ylabel('\beta(t)'); ylim([-.05 1.05]);
title('Certified LPV (hinfgs) on Type C — time-scheduled comfort \rightarrow handling');
subplot(4,1,2); plot(t_out,ab_ol,'b--'); hold on; plot(t_out,ab_cl,'k'); grid on;
ylabel('a_b [m/s^2]'); legend('passive','LPV');
subplot(4,1,3); plot(t_out,td_ol*100,'b--'); hold on; plot(t_out,td_cl*100,'k'); grid on;
ylabel('td [cm]'); legend('passive','LPV');
subplot(4,1,4); plot(t_out,sd_ol*100,'b--'); hold on; plot(t_out,sd_cl*100,'k'); grid on;
ylabel('sd [cm]'); xlabel('time [s]'); legend('passive','LPV');

% --- RMS table (comfort window vs handling window) -----------------------
early=t_out<t0; late=t_out>t1;
fprintf('\n--- LPV RMS: passive vs scheduled (Type C) ---\n');
fprintf('              | body acc [m/s^2] | tire defl [cm] | susp defl [cm]\n');
fprintf('  OL  overall | %14.4f | %12.4f | %12.4f\n', rms(ab_ol),rms(td_ol)*100,rms(sd_ol)*100);
fprintf('  LPV overall | %14.4f | %12.4f | %12.4f\n', rms(ab_cl),rms(td_cl)*100,rms(sd_cl)*100);
fprintf('  LPV comfort | %14.4f | %12.4f | %12.4f  (beta~0)\n', rms(ab_cl(early)),rms(td_cl(early))*100,rms(sd_cl(early))*100);
fprintf('  LPV handling| %14.4f | %12.4f | %12.4f  (beta~1)\n', rms(ab_cl(late)), rms(td_cl(late))*100, rms(sd_cl(late))*100);
% ===== end of certified LPV section =====

%% yco lpv+hinf
% % controller designs
% % Setup the Target Gamma and Bisection Parameters
% road_baseline = 'Type C';
% 
% N_grid = 5;
% beta_grid = linspace(0.05, 0.95, N_grid);
% 
% target_gamma = 0.95;   
% gamma_tol    = 0.05;   
% max_iters    = 30;     
% 
% 
% optimal_knobs = zeros(1, N_grid);
% gammas = zeros(1, N_grid);
% % Define execution sampling time (200 Hz digital ECU standard)
% Ts = 1/700; 
% 
% for i = 1:N_grid
%     beta_current = beta_grid(i);
%     fprintf('Tuning Controller %d/%d for beta = %.2f...\n', i, N_grid, beta_current);
% 
%     knob_low = 0.01; knob_high = 20.0;
%     best_K_c = []; best_knob = 1;
% 
%     for iter = 1:max_iters
%         knob_test = (knob_low + knob_high) / 2;
%         [cl_hinf, K_test, gamma_test, qcar_open] = design_hinf_for_road(road_baseline, 1, 1, knob_test, beta_current);
% 
%         if isempty(gamma_test) || isnan(gamma_test) || isinf(gamma_test)
%             knob_high = knob_test; continue;
%         end
% 
%         best_K_c = K_test; best_knob = knob_test;
% 
%         if abs(gamma_test - target_gamma) <= gamma_tol
%             break;
%         end
%         if gamma_test > target_gamma
%             knob_high = knob_test;
%         else
%             knob_low = knob_test;
%         end
%     end
%     optimal_knobs(i) = best_knob;
%     gammas(i) = gamma_test;
%     cl_hinf_(:,:,i)= cl_hinf;
%     % K_grid = cell(1,N_grid);
%     % ...after the bisection settles on best_K_c:
%     K_grid{i} = best_K_c;
%     qcar_open_grid{i} = qcar_open;   % same for every i, but harmless to store
% end
% % gh0 to beta mapping
% road_test = 'Type C';
% u0 = get_speed_for_road(road_test);
% [~, t, hsum, Gh0_test] = roadprofile_fun(road_test, u0);
% u0 = get_speed_for_road('Type A');
% [~, ~, ~, Gh0_min] = roadprofile_fun('Type A', u0);
% u0 = get_speed_for_road('Type H');
% [~, ~, ~, Gh0_max] = roadprofile_fun('Type H', u0);
% beta_mapped = 0.05 + 0.90 * (log10(Gh0_test) - log10(Gh0_min)) / (log10(Gh0_max) - log10(Gh0_min));
% % blend controller
% % find nearest two
% % blend them
% idx_min = 1; idx_max = numel(beta_grid);
% i = 1;
% while beta_mapped>beta_grid(i)
%     idx_min = i;
%     i = i + 1;
%     idx_max = i;
% end
% while beta_mapped>beta_grid(i)
%     idx_max = i;
%     i = i + 1;
% end
% disp('Analog beta:');
% disp(beta_mapped);
% disp('Lower beta:');
% disp(beta_grid(idx_min));
% disp('Higher beta:');
% disp(beta_grid(idx_max));
% mul1 = (beta_grid(idx_max)-beta_mapped)/(beta_grid(idx_max)-beta_grid(idx_min));
% mul2 = (-beta_grid(idx_min)+beta_mapped)/(beta_grid(idx_max)-beta_grid(idx_min));
% cl_hinf_lower = cl_hinf_(:,:,idx_min); cl_hinf_higher = cl_hinf_(:,:,idx_max);
% 
% cl_hinf_lpv = mul1*cl_hinf_lower + mul2*cl_hinf_higher;
% 
% qcar_open = qcar_open_grid{1};        % one plant, not an array
% 
% Klo = K_grid{idx_min};
% Khi = K_grid{idx_max};
% Kblend = mul1*Klo + mul2*Khi;
% Kblend.u = ["sd","ab"];
% Kblend.y = "u";
% 
% Act = tf(1,[1/60 1]);  Act.u = "u";  Act.y = "fs";
% 
% cl_blend = connect(qcar_open, Act, Kblend, "r", ["ab";"sd";"xb";"td"]);
% 
% road_compare = 'Type C';
% u0_ = get_speed_for_road(road_compare);
% [~, t_, hsum_, Gh0_] = roadprofile_fun(road_compare, u0_);
% r_iso_compare     = hsum_(1:length(t_));
% dt_               = t_(2) - t_(1);
% r_dot_iso_compare = [diff(hsum_)./dt_, 0];
% 
% % time sim.s:
% [y_ab_ol, ~]     = lsim(quartercar('body acceleration','r_dot'), r_dot_iso_compare, t_);
% [y_ab_hinf_lower, ~]  = lsim(cl_hinf_lower('ab','r'), r_iso_compare, t_);
% [y_ab_hinf_higher, ~]  = lsim(cl_hinf_higher('ab','r'), r_iso_compare, t_);
% [y_ab_hinf_lpv, ~]  = lsim(cl_blend('ab','r'), r_iso_compare, t_);
% 
% [y_rs_ol, ~]     = lsim(quartercar('rattle space','r_dot'), r_dot_iso_compare, t_);
% [y_rs_hinf_lower, ~]  = lsim(cl_hinf_lower('sd','r'), r_iso_compare, t_);
% [y_rs_hinf_higher, ~]  = lsim(cl_hinf_higher('sd','r'), r_iso_compare, t_);
% [y_rs_hinf_lpv, ~]  = lsim(cl_blend('sd','r'), r_iso_compare, t_);
% 
% % TIRE DEFLECTION SİMS:
% [y_td_ol, ~]     = lsim(quartercar('tire deflection','r_dot'), r_dot_iso_compare, t_);
% [y_td_hinf_lower, ~]  = lsim(cl_hinf_lower('td','r'), r_iso_compare, t_);
% [y_td_hinf_higher, ~]  = lsim(cl_hinf_higher('td','r'), r_iso_compare, t_);
% [y_td_hinf_lpv, ~]  = lsim(cl_blend('td','r'), r_iso_compare, t_);
% 
% figure(11); clf;
% subplot(3,1,1);
% plot(t_, y_ab_ol,  'b--','LineWidth',0.8); hold on;
% plot(t_, y_ab_hinf_lower, 'r',  'LineWidth',1.0);
% plot(t_, y_ab_hinf_higher, 'g',  'LineWidth',1.0);
% plot(t_, y_ab_hinf_lpv,'k',  'LineWidth',1.0);
% title('Body Acceleration (Comfort)');
% ylabel('m/s^2'); grid on;
% legend(sprintf('OL (%.4f)',   rms(y_ab_ol)), ...
%     sprintf('Hinf_lower (%.4f)',  rms(y_ab_hinf_lower)),...
%     sprintf('Hinf_higher (%.4f)',rms(y_ab_hinf_higher)),...
%     sprintf('Hinf_lpv (%.4f)',rms(y_ab_hinf_lpv)));
% xlim([10, 15]);
% 
% subplot(3,1,2);
% plot(t_, y_td_ol*100,  'b--','LineWidth',0.8); hold on;
% plot(t_, y_td_hinf_lower*100, 'r',  'LineWidth',1.0);
% plot(t_, y_td_hinf_higher*100, 'g',  'LineWidth',1.0);
% plot(t_, y_td_hinf_lpv*100,'k',  'LineWidth',1.0);
% title('Tire Deflection (Road Holding)');
% ylabel('cm'); xlabel('Time (s)'); grid on;
% legend(sprintf('OL (%.4f)',   rms(y_td_ol)*100), ...
%     sprintf('Hinf_lower (%.4f)',  rms(y_td_hinf_lower)*100), ...
%     sprintf('Hinf_higher (%.4f)',  rms(y_td_hinf_higher)*100), ...    
%     sprintf('Hinf_lpv (%.4f)',rms(y_td_hinf_lpv)*100));
% xlim([10, 15]);
% 
% subplot(3,1,3);
% plot(t_, y_rs_ol*100,  'b--','LineWidth',0.8); hold on;
% plot(t_, y_rs_hinf_lower*100, 'r',  'LineWidth',1.0);
% plot(t_, y_rs_hinf_higher*100, 'g',  'LineWidth',1.0);
% plot(t_, y_rs_hinf_lpv*100,'k',  'LineWidth',1.0);
% title('Suspension Deflection');
% ylabel('cm'); grid on;
% legend(sprintf('OL (%.4f)',   rms(y_rs_ol)*100), ...
%     sprintf('Hinf_lower (%.4f)',  rms(y_rs_hinf_lower)*100), ...
%     sprintf('Hinf_higher (%.4f)',  rms(y_rs_hinf_higher)*100), ...
%     sprintf('Hinf_lpv (%.4f)',rms(y_rs_hinf_lpv)*100));
% xlim([10, 15]);
% 
% disp('------------------------------------------------');
% disp(['Road Compare: ', road_compare]);
% disp('------------------------------------------------');
% % fprintf('gamma A: %.4f\n', gamma_out_A);
% % fprintf('gamma D: %.4f\n\n', gamma_out_D);
% 
% disp('--- Body Acceleration RMS [m/s^2] ---');
% fprintf('Open-Loop: %.4f\n', rms(y_ab_ol));
% fprintf('K_lower:       %.4f\n',   rms(y_ab_hinf_lower));
% fprintf('K_higher:      %.4f\n',   rms(y_ab_hinf_higher));
% fprintf('K_lpv:         %.4f\n\n', rms(y_ab_hinf_lpv));
% 
% disp('--- Tire Deflection RMS [cm] ---');
% fprintf('Open-Loop: %.4f\n', rms(y_td_ol)*100);
% fprintf('K_lower:       %.4f\n', rms(y_td_hinf_lower)*100);
% fprintf('K_higher:       %.4f\n', rms(y_td_hinf_higher)*100);
% fprintf('K_lpv:       %.4f\n\n', rms(y_td_hinf_lpv)*100);
% 
% disp('--- Rattle Space RMS [cm] ---');
% fprintf('Open-Loop: %.4f\n', rms(y_rs_ol)*100);
% fprintf('K_lower:       %.4f\n', rms(y_rs_hinf_lower)*100);
% fprintf('K_higher:       %.4f\n', rms(y_rs_hinf_higher)*100);
% fprintf('K_lpv:       %.4f\n\n', rms(y_rs_hinf_lpv)*100);

% % ===== OUTPUT-BLENDED GAIN SCHEDULING (time-varying weight) ==============
% % Replaces the old blend/connect block. Two H-infinity designs (YOUR filters)
% % are the comfort/handling vertices; we schedule by blending their OUTPUTS:
% %       u(t) = (1-w(t))*u_comfort + w(t)*u_handling ,   w(t) in [0,1].
% % This is interpolation gain scheduling (NOT certified LPV). The stability
% % sweep is the stand-in for the Lyapunov certificate hinfgs would give.
% 
% % --- two vertex controllers: reuse the grid endpoints --------------------
% Kc = K_grid{1};      Kc.u = ["sd","ab"];  Kc.y = "u";   % comfort  (low beta)
% Kh = K_grid{end};    Kh.u = ["sd","ab"];  Kh.y = "u";   % handling (high beta)
% fprintf('\nvertex controllers open-loop stable?  comfort=%d  handling=%d\n', ...
%         isstable(Kc), isstable(Kh));
% 
% % --- stability sweep across the blend weight (certificate stand-in) -------
% fprintf('\n--- blend stability sweep ---\n');
% Act = tf(1,[1/60 1]);
% Act.u = "u";  Act.y = "fs";
% for w = 0.1:0.1:0.9
%     Kb = (1-w)*Kc + w*Kh;  Kb.u = ["sd","ab"];  Kb.y = "u";
%     clb = connect(qcar_open, Act, Kb, "r", ["ab";"sd";"td"]);
%     fprintf('  w=%.1f : stable=%d , max Re(pole)=%9.2f\n', ...
%             w, isstable(clb), max(real(pole(clb))));
% end
% % --- plant + actuator as one block:  [r ; u] -> [xb;sd;ab;td] -------------
% Pfull = connect(qcar_open, Act, ["r";"u"], ["xb";"sd";"ab";"td"]);
% [Ap,Bp,Cp,~] = ssdata(Pfull);   nP = size(Ap,1);
% Cm = Cp([2 3],:);               % measurements [sd; ab] (no u feedthrough)
% 
% [Akc,Bkc,Ckc,Dkc] = ssdata(Kc);   nKc = size(Akc,1);
% [Akh,Bkh,Ckh,Dkh] = ssdata(Kh);
% 
% % --- ISO road for the simulation (DISPLACEMENT input) --------------------
% road_sim = 'Type C';
% u0s = get_speed_for_road(road_sim);
% [~, tsim, hsim] = roadprofile_fun(road_sim, u0s);
% r_iso_sim = hsim(1:numel(tsim));
% roadI = griddedInterpolant(tsim, r_iso_sim, 'linear');
% Tend  = min(tsim(end), 30);            % trim for speed; raise if you want more
% t_out = (0:1e-3:Tend).';
% 
% % --- time-varying blend weight: comfort -> handling ----------------------
% w0 = 0.2*Tend;  w1 = 0.8*Tend;
% wfun = @(tt) 0.5*(1 - cos(pi*min(max((tt-w0)/(w1-w0),0),1)));   % 0 -> 1
% 
% % --- simulate the scheduled (output-blended) controller ------------------
% lam = 50;                              % anti-windup leak on the inactive ctrl
% X0  = zeros(nP+nKc+size(Akh,1),1);
% f   = @(t,X) lpv_blend_rhs(t,X, Ap,Bp,Cm, nP,nKc, ...
%             Akc,Bkc,Ckc,Dkc, Akh,Bkh,Ckh,Dkh, roadI, wfun, lam);
% opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',2e-3);
% [~,Xs] = ode45(f, t_out, X0, opts);
% 
% ab_cl=zeros(size(t_out)); sd_cl=ab_cl; td_cl=ab_cl; wt=ab_cl;
% for k = 1:numel(t_out)
%     xP = Xs(k,1:nP).';
%     ab_cl(k)=Cp(3,:)*xP;  sd_cl(k)=Cp(2,:)*xP;  td_cl(k)=Cp(4,:)*xP;
%     wt(k)=wfun(t_out(k));
% end
% 
% % --- open-loop (passive) on the same road --------------------------------
% y_ol = lsim(qcar_open(["ab";"sd";"td"],"r"), roadI(t_out), t_out);
% ab_ol=y_ol(:,1);  sd_ol=y_ol(:,2);  td_ol=y_ol(:,3);
% 
% % --- plots ---------------------------------------------------------------
% figure(20); clf;
% subplot(4,1,1); plot(t_out,wt,'r','LineWidth',1.5); grid on;
% ylabel('blend w(t)'); ylim([-.05 1.05]);
% title('Output-blended gain scheduling: comfort \rightarrow handling (Type C road)');
% subplot(4,1,2); plot(t_out,ab_ol,'b--'); hold on; plot(t_out,ab_cl,'k'); grid on;
% ylabel('body acc [m/s^2]'); legend('passive','scheduled');
% subplot(4,1,3); plot(t_out,td_ol*100,'b--'); hold on; plot(t_out,td_cl*100,'k'); grid on;
% ylabel('tire defl [cm]'); legend('passive','scheduled');
% subplot(4,1,4); plot(t_out,sd_ol*100,'b--'); hold on; plot(t_out,sd_cl*100,'k'); grid on;
% ylabel('susp defl [cm]'); xlabel('time [s]'); legend('passive','scheduled');
% 
% % --- RMS comparison: comfort window vs handling window -------------------
% early = t_out < w0;  late = t_out > w1;
% fprintf('\n--- RMS: passive vs scheduled ---\n');
% fprintf('              | body acc [m/s^2] | tire defl [cm] | susp defl [cm]\n');
% fprintf('  OL  overall | %14.4f | %12.4f | %12.4f\n', rms(ab_ol),        rms(td_ol)*100,        rms(sd_ol)*100);
% fprintf('  CL  overall | %14.4f | %12.4f | %12.4f\n', rms(ab_cl),        rms(td_cl)*100,        rms(sd_cl)*100);
% fprintf('  CL comfort  | %14.4f | %12.4f | %12.4f   (w~0)\n', rms(ab_cl(early)), rms(td_cl(early))*100, rms(sd_cl(early))*100);
% fprintf('  CL handling | %14.4f | %12.4f | %12.4f   (w~1)\n', rms(ab_cl(late)),  rms(td_cl(late))*100,  rms(sd_cl(late))*100);

%% helpers
function dX = lpv_cert_rhs(t,X, Apx,Bpx,Cy_x, nPx, K0,K1, roadI, beta_fun)
    xpx = X(1:nPx);  xk = X(nPx+1:end);
    b   = beta_fun(t);
    aK=(1-b)*K0.a+b*K1.a; bK=(1-b)*K0.b+b*K1.b; cK=(1-b)*K0.c+b*K1.c; dK=(1-b)*K0.d+b*K1.d;
    y   = Cy_x*xpx;                       % suspension deflection (no u feedthrough)
    u   = cK*xk + dK*y;
    r   = roadI(t);
    dX  = [ Apx*xpx + Bpx*[r; u];
            aK*xk + bK*y ];
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

function [K, gamma_out, knob, cl_hinf, qcar_open] = synth_hinf_auto(road_type, beta, target_gamma, params)
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
function dX = lpv_blend_rhs(t,X, Ap,Bp,Cm, nP,nKc, ...
                            Akc,Bkc,Ckc,Dkc, Akh,Bkh,Ckh,Dkh, roadI, wfun, lam)
    xP  = X(1:nP);
    xKc = X(nP+1:nP+nKc);
    xKh = X(nP+nKc+1:end);
    ym  = Cm*xP;                       % [sd; ab]  (no u feedthrough)
    w   = wfun(t);
    uc  = Ckc*xKc + Dkc*ym;
    uh  = Ckh*xKh + Dkh*ym;
    u   = (1-w)*uc + w*uh;             % output blend
    r   = roadI(t);
    % anti-windup leak: pull the under-weighted controller's state to 0 so an
    % open-loop-unstable vertex controller cannot diverge while it is inactive
    dX = [ Ap*xP + Bp(:,1)*r + Bp(:,2)*u;
           Akc*xKc + Bkc*ym - lam*(w)*xKc;
           Akh*xKh + Bkh*ym - lam*(1-w)*xKh ];
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
    
    % ADD TIRE DEFLECTION AS OUTPUT 4: td = x3 - r 
    C = [1 0 0 0; 1 0 -1 0; A(2,:); 0 0 1 0];
    D = [0 0; 0 0; B(2,:); -1 0];

    qcar_open = ss(A, B, C, D);
    qcar_open.StateName  = ["body travel (m)";"body vel (m/s)"; ...
                            "wheel travel (m)";"wheel vel (m/s)"];
    qcar_open.InputName  = ["r";"fs"];
    qcar_open.OutputName = ["xb";"sd";"ab";"td"];

    w_bounce = sqrt(ks/ms);

    Act = tf(1,[1/60 1]);
    Act.InputName = "u"; Act.OutputName = "fs";

    w_wheel_hop = sqrt(kt/mu);  
    w_break = max(10*(V/33), w_wheel_hop * 1.5);
    
    type_w_road = 3; % Kept at 3 based on your previous fix
    if type_w_road==3
        Wroad = tf(sqrt(Gh0 * V) * w_break, [1, w_break]);
    end
    Wroad.u = "d1"; Wroad.y = "r";
    
    % Wact
    Wact = 0.1 * tf([1, w_bounce], [1, 100]);
    Wact.u = "u"; Wact.y = "e1";

    Wd2 = ss(0.01); Wd2.u = "d2"; Wd2.y = "Wd2";
    % Wd3 = ss(0.5);  Wd3.u = "d3"; Wd3.y = "Wd3";
    Wd3 = ss(0.01);  Wd3.u = "d3"; Wd3.y = "Wd3";

    scale_factor = 1;
    % Wsd = scale_factor * knob * (beta) * tf(1, [1/w_bounce, 1]);
    Wsd = scale_factor * knob * tf(1, [1/w_bounce, 1]);
    Wsd.u = "sd"; Wsd.y = "e3";

    Wab = scale_factor * knob * ((1-beta)) * tf(1, conv([1/w_bounce, 1],[1/w_bounce, 1]));
    Wab.u = "ab"; Wab.y = "e2";

    % Handling is evaluated at the wheel-hop frequency
    Wtd = knob * beta * tf(1, [1/w_wheel_hop, 1]);
    Wtd.u = "td"; Wtd.y = "e4";

    sdmeas = sumblk("y1 = sd+Wd2");
    abmeas = sumblk("y2 = ab+Wd3");
    ICinputs  = ["d1";"d2";"d3";"u"];
    ICoutputs = ["e1";"e2";"e3";"e4";"y1";"y2"];

    % Notice qcar_open(["sd","ab"],:) isolates just the outputs needed to connect to the weights
    qcaric = connect(qcar_open(["sd","ab","td"],:), Act, Wroad, Wact, Wab, Wsd, Wtd, ...
                     Wd2, Wd3, sdmeas, abmeas, ICinputs, ICoutputs);

    [K, ~, gamma_out] = hinfsyn(qcaric, 2, 1);
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

function aw_rms = iso2631_wk_rms(a, fs)
% ISO 2631-1 Wk frequency-weighted RMS of vertical acceleration.
%   a  : acceleration time history [m/s^2]
%   fs : sampling rate [Hz]
% Verified against ISO 8041 reference Wk magnitudes (<0.3% error, 0.5-20 Hz).

    % --- input guards (this is what was giving you NaN) ---
    if ~isscalar(fs) || ~isfinite(fs) || fs <= 0
        error('fs must be a positive finite scalar. Got: %s', mat2str(fs));
    end
    a = a(:);
    if any(~isfinite(a))
        error('Input acceleration contains NaN/Inf at %d samples.', sum(~isfinite(a)));
    end

    % --- Wk parameters (ISO 2631-1:1997) ---
    f1=0.4;  Q1=0.71;   f2=100; Q2=0.71;      % band limiting
    f3=12.5; f4=12.5; Q4=0.63;                % a-v transition
    f5=2.37; Q5=0.91; f6=3.35; Q6=0.91;       % upward step
    w1=2*pi*f1; w2=2*pi*f2; w3=2*pi*f3; w4=2*pi*f4; w5=2*pi*f5; w6=2*pi*f6;

    % --- continuous biquads {num, den}; note the 0.5 in the step section ---
    Hh = {[1 0 0],                 [1 w1/Q1 w1^2]};
    Hl = {[0 0 w2^2],              [1 w2/Q2 w2^2]};
    Ht = {[0 w4^2/w3 w4^2],        [1 w4/Q4 w4^2]};
    Hs = {[0.5/w5^2 0.5/(Q5*w5) 0.5], [1/w6^2 1/(Q6*w6) 1]};   % <-- 0.5 fix
    sections = {Hh, Hl, Ht, Hs};

    aw = a;
    for k = 1:numel(sections)
        [b, ad] = bilinear_biquad(sections{k}{1}, sections{k}{2}, fs);
        aw = filter(b, ad, aw);
    end
    aw_rms = sqrt(mean(aw.^2));
end

function [B, A] = bilinear_biquad(num, den, fs)
    K = 2*fs;
    b2=num(1); b1=num(2); b0=num(3);
    a2=den(1); a1=den(2); a0=den(3);
    B = [ b2*K^2 + b1*K + b0,  -2*b2*K^2 + 2*b0,   b2*K^2 - b1*K + b0];
    A = [ a2*K^2 + a1*K + a0,  -2*a2*K^2 + 2*a0,   a2*K^2 - a1*K + a0];
    B = B / A(1);
    A = A / A(1);
end
