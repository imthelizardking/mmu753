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
    case 'Type A',  u0 = 33;
    case 'Type B',  u0 = 30;
    case 'Type C',  u0 = 25;
    case 'Type D',  u0 = 20;
    case 'Type E',  u0 = 15;
    case 'Type F',  u0 = 10;
    case 'Type G',  u0 = 7;
    case 'Type H',  u0 = 5;
        % ── Airfield ───────────────────────────────────────────────────
    case 'Smooth airfield runway', u0 = 33;
    case 'Rough airfield runway',  u0 = 20;
        % ── Paved Roads ────────────────────────────────────────────────
    case 'Tarmac motorway',        u0 = 33;
    case 'Concrete motorway',      u0 = 33;
    case 'Good road',              u0 = 30;
    case 'Smooth highway',         u0 = 30;
    case 'Average road',           u0 = 25;
    case 'Minor road',             u0 = 20;
    case 'Highway with gravel',    u0 = 18;
    case 'Poor road',              u0 = 15;
    case 'Very rough unmade road', u0 = 10;
        % ── Off-road ───────────────────────────────────────────────────
    case 'Pasture',                        u0 = 12;
    case 'Cross country medium roughness', u0 = 8;
    case 'Grassland',                      u0 = 8;
    case 'Rocky Soil',                     u0 = 5;
    case 'Cross country severe',           u0 = 4;
        % ── Test Courses ───────────────────────────────────────────────
    case 'Random Test Course', u0 = 8;
    case 'Rocky Test Course',  u0 = 5;
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
bodemag(qcar(["sd","ab"],"r"), 'b--', opts); hold on
bodemag(quartercar_cl_hinf(["sd","ab"],"r"), 'k');
grid minor
legend('Open-loop','H-infinity','location','SouthEast')
title('Open-loop vs H-infiinty Bode');
xlim([0.01, 1000])

%% time simulations
[y_ab_ol, ~]      = lsim(quartercar('body acceleration','r_dot'), rdot_iso, t);
[y_ab_lqr, ~]     = lsim(quartercar_cl_lqr('body acceleration','r_dot'), rdot_iso, t);
[y_ab_hinf, ~]    = lsim(quartercar_cl_hinf('ab','r'), r_iso, t);
[y_rs_ol, ~]   = lsim(quartercar('rattle space','r_dot'), rdot_iso, t);
[y_rs_lqr, ~]  = lsim(quartercar_cl_lqr('rattle space','r_dot'), rdot_iso, t);
[y_rs_hinf, ~] = lsim(quartercar_cl_hinf('sd','r'), r_iso, t);

figure(3); clf;
subplot(2,1,1);
plot(t, y_ab_ol,  'b--','LineWidth',0.8); hold on;
plot(t, y_ab_lqr, 'r',  'LineWidth',1.0);
plot(t, y_ab_hinf,'k',  'LineWidth',1.0);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_ab_ol)), ...
       sprintf('LQR (%.4f)',  rms(y_ab_lqr)), ...
       sprintf('H-inf (%.4f)',rms(y_ab_hinf)));
xlim([10, 15]);
subplot(2,1,2);
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

%% yco lpv+hinf
% controller designs
% Setup the Target Gamma and Bisection Parameters
road_baseline = 'Type C';

N_grid = 5;
beta_grid = linspace(0.05, 0.95, N_grid);

target_gamma = 0.95;   
gamma_tol    = 0.05;   
max_iters    = 30;     


optimal_knobs = zeros(1, N_grid);
gammas = zeros(1, N_grid);
% Define execution sampling time (200 Hz digital ECU standard)
Ts = 1/700; 

for i = 1:N_grid
    beta_current = beta_grid(i);
    fprintf('Tuning Controller %d/%d for beta = %.2f...\n', i, N_grid, beta_current);
    
    knob_low = 0.01; knob_high = 20.0;
    best_K_c = []; best_knob = 1;
    
    for iter = 1:max_iters
        knob_test = (knob_low + knob_high) / 2;
        [cl_hinf, K_test, gamma_test, qcar_open] = design_hinf_for_road(road_baseline, 1, 1, knob_test, beta_current);
        
        if isempty(gamma_test) || isnan(gamma_test) || isinf(gamma_test)
            knob_high = knob_test; continue;
        end
        
        best_K_c = K_test; best_knob = knob_test;
        
        if abs(gamma_test - target_gamma) <= gamma_tol
            break;
        end
        if gamma_test > target_gamma
            knob_high = knob_test;
        else
            knob_low = knob_test;
        end
    end
    optimal_knobs(i) = best_knob;
    gammas(i) = gamma_test;
    cl_hinf_(:,:,i)= cl_hinf;
    % K_grid = cell(1,N_grid);
    % ...after the bisection settles on best_K_c:
    K_grid{i} = best_K_c;
    qcar_open_grid{i} = qcar_open;   % same for every i, but harmless to store
end
%%
% gh0 to beta mapping
road_test = 'Type C';
u0 = get_speed_for_road(road_test);
[~, t, hsum, Gh0_test] = roadprofile_fun(road_test, u0);
u0 = get_speed_for_road('Type A');
[~, ~, ~, Gh0_min] = roadprofile_fun('Type A', u0);
u0 = get_speed_for_road('Type H');
[~, ~, ~, Gh0_max] = roadprofile_fun('Type H', u0);
beta_mapped = 0.05 + 0.90 * (log10(Gh0_test) - log10(Gh0_min)) / (log10(Gh0_max) - log10(Gh0_min));
% blend controller
% find nearest two
% blend them
idx_min = 1; idx_max = numel(beta_grid);
i = 1;
while beta_mapped>beta_grid(i)
    idx_min = i;
    i = i + 1;
    idx_max = i;
end
while beta_mapped>beta_grid(i)
    idx_max = i;
    i = i + 1;
end
disp('Analog beta:');
disp(beta_mapped);
disp('Lower beta:');
disp(beta_grid(idx_min));
disp('Higher beta:');
disp(beta_grid(idx_max));
mul1 = (beta_grid(idx_max)-beta_mapped)/(beta_grid(idx_max)-beta_grid(idx_min));
mul2 = (-beta_grid(idx_min)+beta_mapped)/(beta_grid(idx_max)-beta_grid(idx_min));
cl_hinf_lower = cl_hinf_(:,:,idx_min); cl_hinf_higher = cl_hinf_(:,:,idx_max);

cl_hinf_lpv = mul1*cl_hinf_lower + mul2*cl_hinf_higher;

qcar_open = qcar_open_grid{1};        % one plant, not an array

Klo = K_grid{idx_min};
Khi = K_grid{idx_max};
Kblend = mul1*Klo + mul2*Khi;
Kblend.u = ["sd","ab"];
Kblend.y = "u";

Act = tf(1,[1/60 1]);  Act.u = "u";  Act.y = "fs";

cl_blend = connect(qcar_open, Act, Kblend, "r", ["ab";"sd";"xb";"td"]);

road_compare = 'Type C';
u0_ = get_speed_for_road(road_compare);
[~, t_, hsum_, Gh0_] = roadprofile_fun(road_compare, u0_);
r_iso_compare     = hsum_(1:length(t_));
dt_               = t_(2) - t_(1);
r_dot_iso_compare = [diff(hsum_)./dt_, 0];

% time sim.s:
[y_ab_ol, ~]     = lsim(quartercar('body acceleration','r_dot'), r_dot_iso_compare, t_);
[y_ab_hinf_lower, ~]  = lsim(cl_hinf_lower('ab','r'), r_iso_compare, t_);
[y_ab_hinf_higher, ~]  = lsim(cl_hinf_higher('ab','r'), r_iso_compare, t_);
[y_ab_hinf_lpv, ~]  = lsim(cl_blend('ab','r'), r_iso_compare, t_);

[y_rs_ol, ~]     = lsim(quartercar('rattle space','r_dot'), r_dot_iso_compare, t_);
[y_rs_hinf_lower, ~]  = lsim(cl_hinf_lower('sd','r'), r_iso_compare, t_);
[y_rs_hinf_higher, ~]  = lsim(cl_hinf_higher('sd','r'), r_iso_compare, t_);
[y_rs_hinf_lpv, ~]  = lsim(cl_blend('sd','r'), r_iso_compare, t_);

% TIRE DEFLECTION SİMS:
[y_td_ol, ~]     = lsim(quartercar('tire deflection','r_dot'), r_dot_iso_compare, t_);
[y_td_hinf_lower, ~]  = lsim(cl_hinf_lower('td','r'), r_iso_compare, t_);
[y_td_hinf_higher, ~]  = lsim(cl_hinf_higher('td','r'), r_iso_compare, t_);
[y_td_hinf_lpv, ~]  = lsim(cl_blend('td','r'), r_iso_compare, t_);

figure(11); clf;
subplot(3,1,1);
plot(t_, y_ab_ol,  'b--','LineWidth',0.8); hold on;
plot(t_, y_ab_hinf_lower, 'r',  'LineWidth',1.0);
plot(t_, y_ab_hinf_higher, 'g',  'LineWidth',1.0);
plot(t_, y_ab_hinf_lpv,'k',  'LineWidth',1.0);
title('Body Acceleration (Comfort)');
ylabel('m/s^2'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_ab_ol)), ...
    sprintf('Hinf_lower (%.4f)',  rms(y_ab_hinf_lower)),...
    sprintf('Hinf_higher (%.4f)',rms(y_ab_hinf_higher)),...
    sprintf('Hinf_lpv (%.4f)',rms(y_ab_hinf_lpv)));
xlim([10, 15]);

subplot(3,1,2);
plot(t_, y_td_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t_, y_td_hinf_lower*100, 'r',  'LineWidth',1.0);
plot(t_, y_td_hinf_higher*100, 'g',  'LineWidth',1.0);
plot(t_, y_td_hinf_lpv*100,'k',  'LineWidth',1.0);
title('Tire Deflection (Road Holding)');
ylabel('cm'); xlabel('Time (s)'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_td_ol)*100), ...
    sprintf('Hinf_lower (%.4f)',  rms(y_td_hinf_lower)*100), ...
    sprintf('Hinf_higher (%.4f)',  rms(y_td_hinf_higher)*100), ...    
    sprintf('Hinf_lpv (%.4f)',rms(y_td_hinf_lpv)*100));
xlim([10, 15]);

subplot(3,1,3);
plot(t_, y_rs_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t_, y_rs_hinf_lower*100, 'r',  'LineWidth',1.0);
plot(t_, y_rs_hinf_higher*100, 'g',  'LineWidth',1.0);
plot(t_, y_rs_hinf_lpv*100,'k',  'LineWidth',1.0);
title('Suspension Deflection');
ylabel('cm'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_rs_ol)*100), ...
    sprintf('Hinf_lower (%.4f)',  rms(y_rs_hinf_lower)*100), ...
    sprintf('Hinf_higher (%.4f)',  rms(y_rs_hinf_higher)*100), ...
    sprintf('Hinf_lpv (%.4f)',rms(y_rs_hinf_lpv)*100));
xlim([10, 15]);

disp('------------------------------------------------');
disp(['Road Compare: ', road_compare]);
disp('------------------------------------------------');
% fprintf('gamma A: %.4f\n', gamma_out_A);
% fprintf('gamma D: %.4f\n\n', gamma_out_D);

disp('--- Body Acceleration RMS [m/s^2] ---');
fprintf('Open-Loop: %.4f\n', rms(y_ab_ol));
fprintf('K_lower:       %.4f\n',   rms(y_ab_hinf_lower));
fprintf('K_higher:      %.4f\n',   rms(y_ab_hinf_higher));
fprintf('K_lpv:         %.4f\n\n', rms(y_ab_hinf_lpv));

disp('--- Tire Deflection RMS [cm] ---');
fprintf('Open-Loop: %.4f\n', rms(y_td_ol)*100);
fprintf('K_lower:       %.4f\n', rms(y_td_hinf_lower)*100);
fprintf('K_higher:       %.4f\n', rms(y_td_hinf_higher)*100);
fprintf('K_lpv:       %.4f\n\n', rms(y_td_hinf_lpv)*100);

disp('--- Rattle Space RMS [cm] ---');
fprintf('Open-Loop: %.4f\n', rms(y_rs_ol)*100);
fprintf('K_lower:       %.4f\n', rms(y_rs_hinf_lower)*100);
fprintf('K_higher:       %.4f\n', rms(y_rs_hinf_higher)*100);
fprintf('K_lpv:       %.4f\n\n', rms(y_rs_hinf_lpv)*100);
%% LPV + H-infinity active suspension via gain-scheduled synthesis (hinfgs)
% ------------------------------------------------------------------------
% Instead of synthesising N independent H-inf controllers and blending them
% (which gave unstable blends), we solve ONE gain-scheduled H-inf problem
% over the whole beta range. hinfgs returns a polytopic controller whose
% convex interpolation is certified stable for every admissible beta(t) by a
% single quadratic Lyapunov function.
%
% Requires: Robust Control Toolbox (hinfgs, psys, ltisys, ltiss, psinfo)
%           + your helpers roadprofile_fun, get_speed_for_road,
%           get_vehicle_params on the path.
%
% THREE LEGACY-API SPOTS TO SANITY-CHECK against `doc` if anything errors:
%   (1) hinfgs needs the AFFINE pvec form (psys(pv,[s0 s1])), not polytopic.
%   (2) the derivative term s1's E argument must be SCALAR 0 (not zeros(n)),
%       else hinfgs reports "Not available when the E matrix varies".
%   (3) hinfgs(pdP,[NMEAS NCON]) -> here [2 1]  (2 measurements, 1 control).
%       Vertex order from psinfo/ltiss should match box corners [0,1]; if beta
%       runs backwards in the stability sweep, swap Av/Bv/Cv/Dv{1} and {2}.
% ------------------------------------------------------------------------

% ---- user knobs -------------------------------------------------------
params        = 'nom';      % vehicle parameter set (get_vehicle_params)
road_baseline = 'Type C';   % road used to shape the synthesis weights
road_sim      = 'Type C';   % road used for the time simulation
knob          = 1.0;        % single fixed weight scale (push down for lower gamma)

% ---- physical plant (displacement-input) + actuator -------------------
[ms,mu,bs,ks,kt,bt] = get_vehicle_params(params);

Ap = [ 0 1 0 0; ...
       [-ks -bs ks bs]/ms; ...
       0 0 0 1; ...
       [ks bs -ks-kt -bs]/mu];
Bp = [ 0 0; 0 1e3/ms; 0 0; [kt -1e3]/mu];
Cp = [1 0 0 0; 1 0 -1 0; Ap(2,:); 0 0 1 0];
Dp = [0 0; 0 0; Bp(2,:); -1 0];

qcar_open = ss(Ap,Bp,Cp,Dp);
qcar_open.InputName  = ["r";"fs"];
qcar_open.OutputName = ["xb";"sd";"ab";"td"];

Act = ss(tf(1,[1/60 1]));            % actuator u -> fs (strictly proper)
[Aa,Ba,Ca,Da] = ssdata(Act);         % Da = 0 -> no algebraic loop

% ---- build the generalised plant at a NOMINAL beta, then go affine ----
% qcaric(beta) is affine: only regulated rows e2 (~ (1-beta)) and e4 (~ beta)
% depend on beta. Build it ONCE at beta = 0.5 so P0 and P1 share one basis.
beta_nom = 0.5;
qcaric = build_qcaric(qcar_open, road_baseline, params, knob, beta_nom);

[Am,Bm,Cm,Dm] = ssdata(ss(qcaric));
n = size(Am,1);

% rows of ICoutputs = [e1;e2;e3;e4;y1;y2] -> e2 is row 2, e4 is row 4.
% "unit" coefficient rows (value at beta=0.5 divided by 0.5):
Ce2 = Cm(2,:)/beta_nom;   De2 = Dm(2,:)/beta_nom;
Ce4 = Cm(4,:)/beta_nom;   De4 = Dm(4,:)/beta_nom;

% P0 = value at beta = 0  : e2 at full weight, e4 = 0
C0 = Cm; D0 = Dm;
C0(2,:) = Ce2;  D0(2,:) = De2;
C0(4,:) = 0;    D0(4,:) = 0;

% P1 = d/dbeta            : e2 -> -unit, e4 -> +unit, all else 0
C1 = zeros(size(Cm)); D1 = zeros(size(Dm));
C1(2,:) = -Ce2; D1(2,:) = -De2;
C1(4,:) =  Ce4; D1(4,:) =  De4;

% Affine parameter-dependent form (this is what hinfgs expects: it schedules
% via psinfo(pdP,'par')/polydec on the plant's pvec). Only C,D depend on beta;
% A,B,E are constant. The derivative term s1 MUST pass a SCALAR 0 as its E
% argument -- a zero *matrix* flags a descriptor system and makes hinfgs think
% E varies ("Not available when the E matrix varies").
s0 = ltisys(Am, Bm, C0,      D0);                    % E0 = I (default)
s1 = ltisys(zeros(n), zeros(size(Bm)), C1, D1, 0);   % E1 = 0 via scalar 0
pv  = pvec('box', [0 1]);
pdP = psys(pv, [s0 s1]);                              % affine, constant E = I

% ---- gain-scheduled H-infinity synthesis ------------------------------
[gopt, pdK] = hinfgs(pdP, [2 1]);     % 2 measurements, 1 control
fprintf('hinfgs guaranteed gamma over beta in [0,1]: %.4f\n', gopt);

% pull out the vertex controllers (one per box vertex: beta=0 and beta=1)
[~, nv, nsK] = psinfo(pdK);
Av = cell(1,nv); Bv = cell(1,nv); Cv = cell(1,nv); Dv = cell(1,nv);
for j = 1:nv
    [Av{j},Bv{j},Cv{j},Dv{j}] = ltiss(psinfo(pdK,'sys',j));
end
fprintf('controller order = %d, vertices = %d\n', nsK, nv);

% ---- stability check across beta (this is what failed before) ---------
fprintf('\n--- frozen closed-loop stability sweep ---\n');
for b = 0:0.1:1
    Acl = closed_loop_A(b, Av,Bv,Cv,Dv, Ap,Bp,Cp,Dp, Aa,Ba,Ca);
    fprintf('beta=%.1f : stable=%d , max Re(pole)=%8.3f\n', ...
            b, all(real(eig(Acl))<0), max(real(eig(Acl))));
end

% ---- road for the time simulation -------------------------------------
u0 = get_speed_for_road(road_sim);
[~, t_road, hsum, ~] = roadprofile_fun(road_sim, u0);
r_road = hsum(1:numel(t_road));            % road displacement [m]

% ---- LPV time simulation: beta(t) actually varies ---------------------
nx = 4 + size(Aa,1) + nsK;                 % plant + actuator + controller
X0 = zeros(nx,1);
odef = @(t,X) lpv_rhs(t, X, Av,Bv,Cv,Dv, ...
                      Ap,Bp,Cp,Dp, Aa,Ba,Ca, t_road, r_road);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-3);
[tt, XX] = ode45(odef, [t_road(1) t_road(end)], X0, opts);

% reconstruct outputs (ab, sd, td) and beta(t)
ab = zeros(size(tt)); sd = ab; td = ab; bb = ab;
for k = 1:numel(tt)
    xp = XX(k,1:4).'; xa = XX(k,5:4+size(Aa,1)).';
    fs = Ca*xa;
    sd(k) = Cp(2,:)*xp;
    ab(k) = Cp(3,:)*xp + Dp(3,2)*fs;
    r_k   = interp1(t_road, r_road, tt(k), 'linear', 0);
    td(k) = Cp(4,:)*xp - r_k;
    bb(k) = beta_of_t(tt(k));
end

fprintf('\n--- LPV closed-loop RMS (road %s) ---\n', road_sim);
fprintf('body accel : %.4f m/s^2\n', rms(ab));
fprintf('susp defl  : %.4f cm\n',   rms(sd)*100);
fprintf('tire defl  : %.4f cm\n',   rms(td)*100);

% ---- plots ------------------------------------------------------------
figure; clf;
subplot(4,1,1); plot(tt,ab,'k'); grid on; ylabel('a_b [m/s^2]');
title('LPV+H_\infty active suspension, scheduled on \beta(t)');
subplot(4,1,2); plot(tt,sd*100,'k'); grid on; ylabel('sd [cm]');
subplot(4,1,3); plot(tt,td*100,'k'); grid on; ylabel('td [cm]');
subplot(4,1,4); plot(tt,bb,'r','LineWidth',1.2); grid on;
ylabel('\beta(t)'); xlabel('time [s]'); ylim([-0.05 1.05]);

% ======================== local functions =============================
function qcaric = build_qcaric(qcar_open, road_type, params, knob, beta)
% Reproduces your design_hinf_for_road weighting, parameterised by beta.
    u0 = get_speed_for_road(road_type); V = u0;
    [~, ~, ~, Gh0] = roadprofile_fun(road_type, u0);
    [ms,mu,bs,ks,kt,~] = get_vehicle_params(params); %#ok<ASGLU>

    w_bounce    = sqrt(ks/ms);
    w_wheel_hop = sqrt(kt/mu);
    w_break     = max(10*(V/33), w_wheel_hop*1.5);

    Act = tf(1,[1/60 1]);              Act.u = "u";  Act.y = "fs";
    Wroad = tf(sqrt(Gh0*V)*w_break, [1 w_break]); Wroad.u="d1"; Wroad.y="r";
    Wact  = 0.1*tf([1 w_bounce],[1 100]);          Wact.u ="u";  Wact.y ="e1";
    Wd2 = ss(0.01); Wd2.u="d2"; Wd2.y="Wd2";
    Wd3 = ss(0.01); Wd3.u="d3"; Wd3.y="Wd3";

    Wsd = knob              * tf(1,[1/w_bounce 1]);                    Wsd.u="sd"; Wsd.y="e3";
    Wab = knob*(1-beta)     * tf(1, conv([1/w_bounce 1],[1/w_bounce 1])); Wab.u="ab"; Wab.y="e2";
    Wtd = knob*beta         * tf(1,[1/w_wheel_hop 1]);                 Wtd.u="td"; Wtd.y="e4";

    sdmeas = sumblk("y1 = sd + Wd2");
    abmeas = sumblk("y2 = ab + Wd3");
    ICin   = ["d1";"d2";"d3";"u"];
    ICout  = ["e1";"e2";"e3";"e4";"y1";"y2"];

    qcaric = connect(qcar_open(["sd","ab","td"],:), Act, Wroad, Wact, ...
                     Wab, Wsd, Wtd, Wd2, Wd3, sdmeas, abmeas, ICin, ICout);
end

function [Ak,Bk,Ck,Dk] = Kof(beta, Av,Bv,Cv,Dv)
% Interpolate the two vertex controllers. Vertices are ordered [beta=0, beta=1]
% (same order as sv0,sv1 in psys), so the convex weights are [1-beta, beta].
    c = [1-beta, beta];
    Ak = zeros(size(Av{1})); Bk = zeros(size(Bv{1}));
    Ck = zeros(size(Cv{1})); Dk = zeros(size(Dv{1}));
    for j = 1:numel(Av)
        Ak = Ak + c(j)*Av{j};  Bk = Bk + c(j)*Bv{j};
        Ck = Ck + c(j)*Cv{j};  Dk = Dk + c(j)*Dv{j};
    end
end

function Acl = closed_loop_A(beta, Av,Bv,Cv,Dv, Ap,Bp,Cp,Dp, Aa,Ba,Ca)
% Frozen closed-loop A for [xp; xa; xk] with road set to zero.
    [Ak,Bk,Ck,Dk] = Kof(beta, Av,Bv,Cv,Dv);
    na = size(Aa,1); nk = size(Ak,1);
    Cym = [Cp(2,:); Cp(3,:)];              % measurements [sd; ab] from plant states
    Dyf = [zeros(1,na); Dp(3,2)*Ca];       % ab feedthrough from fs (= Ca*xa)
    Bf  = Bp(:,2)*Ca;                      % fs path into the plant (4 x na)
    Acl = [ Ap,            Bf,                 zeros(4,nk);
            Ba*Dk*Cym,     Aa + Ba*Dk*Dyf,     Ba*Ck;
            Bk*Cym,        Bk*Dyf,             Ak ];
end

function dX = lpv_rhs(t, X, Av,Bv,Cv,Dv, Ap,Bp,Cp,Dp, Aa,Ba,Ca, t_road, r_road)
    na = size(Aa,1);  nk = size(Av{1},1);
    xp = X(1:4);  xa = X(5:4+na);  xk = X(4+na+1:4+na+nk);

    r  = interp1(t_road, r_road, t, 'linear', 0);
    fs = Ca*xa;

    sd = Cp(2,:)*xp;
    ab = Cp(3,:)*xp + Dp(3,2)*fs;
    y  = [sd; ab];

    [Ak,Bk,Ck,Dk] = Kof(beta_of_t(t), Av,Bv,Cv,Dv);
    u  = Ck*xk + Dk*y;

    dxp = Ap*xp + Bp*[r; fs];
    dxa = Aa*xa + Ba*u;
    dxk = Ak*xk + Bk*y;
    dX  = [dxp; dxa; dxk];
end

function beta = beta_of_t(t)
% Scheduling trajectory: comfort -> (smooth) -> handling.
    if t < 4
        beta = 0.05;
    elseif t <= 6
        beta = 0.05 + 0.90*0.5*(1 - cos(pi*(t-4)/2));
    else
        beta = 0.95;
    end
end
%% helpers
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
        case 'Type A',  u0 = 33;
        case 'Type B',  u0 = 50;%30;
        case 'Type C',  u0 = 19;%25;
        case 'Type D',  u0 = 7;%20;
        case 'Type E',  u0 = 5;%15;
        case 'Type F',  u0 = 2;%10;
        case 'Type G',  u0 = 1.5;%7;
        case 'Type H',  u0 = 1;%5;
        case 'Smooth airfield runway',          u0 = 33;
        case 'Rough airfield runway',           u0 = 20;
        case 'Tarmac motorway',                 u0 = 33;
        case 'Concrete motorway',               u0 = 33;
        case 'Good road',                       u0 = 30;
        case 'Smooth highway',                  u0 = 30;
        case 'Average road',                    u0 = 25;
        case 'Minor road',                      u0 = 20;
        case 'Highway with gravel',             u0 = 18;
        case 'Poor road',                       u0 = 15;
        case 'Very rough unmade road',          u0 = 10;
        case 'Pasture',                         u0 = 12;
        case 'Cross country medium roughness',  u0 = 8;
        case 'Grassland',                       u0 = 8;
        case 'Rocky Soil',                      u0 = 5;
        case 'Cross country severe',            u0 = 4;
        case 'Random Test Course',              u0 = 8;
        case 'Rocky Test Course',               u0 = 5;
        otherwise
            error('Unknown road type: "%s"', road_type);
    end
end

function [Gh0, V] = get_road_severity(road_type)
    switch road_type
        case 'Type A',  Gh0 = 1     * 1e-6; V = 33;
        case 'Type B',  Gh0 = 4     * 1e-6; V = 30;
        case 'Type C',  Gh0 = 16    * 1e-6; V = 25;
        case 'Type D',  Gh0 = 64    * 1e-6; V = 20;
        case 'Type E',  Gh0 = 256   * 1e-6; V = 15;
        case 'Type F',  Gh0 = 1024  * 1e-6; V = 10;
        case 'Type G',  Gh0 = 4096  * 1e-6; V = 7;
        case 'Type H',  Gh0 = 16384 * 1e-6; V = 5;
        case 'Smooth airfield runway', Gh0 = (4.3e-11)/(2*pi); V = 33;
        case 'Rough airfield runway',  Gh0 = (8.1e-6) /(2*pi); V = 20;
        case 'Tarmac motorway',        Gh0 = (6.45e-8)/(2*pi); V = 33;
        case 'Concrete motorway',      Gh0 = (2.14e-7)/(2*pi); V = 33;
        case 'Good road',              Gh0 = (3.0e-7) /(2*pi); V = 30;
        case 'Smooth highway',         Gh0 = (4.8e-7) /(2*pi); V = 30;
        case 'Average road',           Gh0 = (2.0e-6) /(2*pi); V = 25;
        case 'Minor road',             Gh0 = (4.39e-6)/(2*pi); V = 20;
        case 'Highway with gravel',    Gh0 = (4.4e-6) /(2*pi); V = 18;
        case 'Poor road',              Gh0 = (1.5e-5) /(2*pi); V = 15;
        case 'Very rough unmade road', Gh0 = (5.17e-5)/(2*pi); V = 10;
        case 'Pasture',                        Gh0 = (1.2e-5) /(2*pi); V = 12;
        case 'Cross country medium roughness', Gh0 = (3.16e-5)/(2*pi); V = 8;
        case 'Grassland',                      Gh0 = (4.98e-5)/(2*pi); V = 8;
        case 'Rocky Soil',                     Gh0 = (2.34e-4)/(2*pi); V = 5;
        case 'Cross country severe',           Gh0 = (3.6e-4) /(2*pi); V = 4;
        case 'Random Test Course', Gh0 = (3.44e-4)/(2*pi); V = 8;
        case 'Rocky Test Course',  Gh0 = (1.01e-3)/(2*pi); V = 5;
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

function dX = lpv_dynamics(t, X, Ap, Bp, Cp, Dp, Ak, Bk, Ck, Dk, beta_grid, t_road, hsum_road)
    % 1. Extract Plant State
    xp = X(1:4);
    
    % 2. Calculate Road Input and Beta at current time t
    r_t = interp1(t_road, hsum_road, t, 'linear', 0);
    beta_t = get_beta_trajectory(t);
    
    % 3. Calculate Interpolation Weights for the 5 controllers
    weights = zeros(1, 5);
    for i = 1:5
        dist = abs(beta_grid(i) - beta_t) / (beta_grid(2) - beta_grid(1));
        weights(i) = max(0, 1 - dist);
    end
    weights = weights / sum(weights); % Normalize
    
    % 4. Measure Plant Outputs [sd; ab]
    y_meas = Cp(1:2, :) * xp + Dp(1:2, 1) * r_t;
    
    % 5. Calculate Blended Control Force (u)
    u_final = 0;
    idx = 5;
    xk = cell(1, 5);
    for i = 1:5
        xk{i} = X(idx : idx+5); % Extract the 6 states for this controller
        u_final = u_final + weights(i) * (Ck{i} * xk{i} + Dk{i} * y_meas);
        idx = idx + 6;
    end
    
    % 6. State Derivatives
    dX = zeros(34, 1);
    dX(1:4) = Ap * xp + Bp * [r_t; u_final]; % Plant dynamics
    
    % Controller dynamics with Anti-Windup Decay
    idx = 5;
    for i = 1:5
        decay = -20 * (1 - weights(i)) * xk{i}; 
        dX(idx : idx+5) = Ak{i} * xk{i} + Bk{i} * y_meas + decay;
        idx = idx + 6;
    end
end

function beta = get_beta_trajectory(t)
    % 0 to 4s: Comfort mode
    % 4 to 6s: Smooth transition
    % 6s onwards: Handling mode
    if t < 4.0
        beta = 0.05;
    elseif t >= 4.0 && t <= 6.0
        phase = (t - 4.0) / 2.0; 
        beta = 0.05 + 0.90 * 0.5 * (1 - cos(pi * phase));
    else
        beta = 0.95;
    end
end
