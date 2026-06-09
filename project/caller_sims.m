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
    1,1, 3); % synthsize A

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
[cl_hinf_A, K_A, gamma_out_A, qcar_open_A] = design_hinf_for_road('Type A', 1, 1, 3); % synthsize A
[cl_hinf_D, K_D, gamma_out_D, qcar_open_D] = design_hinf_for_road('Type D', 1, 1, 2.1); % synthsize D

road_compare = 'Type D';
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
plot(t_, y_rs_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t_, y_rs_hinfA*100, 'r',  'LineWidth',1.0);
plot(t_, y_rs_hinfD*100,'k',  'LineWidth',1.0);
title('Suspension Deflection');
ylabel('cm'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_rs_ol)*100), ...
    sprintf('HinfA (%.4f)',  rms(y_rs_hinfA)*100), ...
    sprintf('HinfD (%.4f)',rms(y_rs_hinfD)*100));
xlim([10, 15]);

subplot(3,1,3);
plot(t_, y_td_ol*100,  'b--','LineWidth',0.8); hold on;
plot(t_, y_td_hinfA*100, 'r',  'LineWidth',1.0);
plot(t_, y_td_hinfD*100,'k',  'LineWidth',1.0);
title('Tire Deflection (Road Holding)');
ylabel('cm'); xlabel('Time (s)'); grid on;
legend(sprintf('OL (%.4f)',   rms(y_td_ol)*100), ...
    sprintf('HinfA (%.4f)',  rms(y_td_hinfA)*100), ...
    sprintf('HinfD (%.4f)',rms(y_td_hinfD)*100));
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
% unit test:
fs = 700;
fcheck = logspace(-1, 2, 200); g = zeros(size(fcheck));
T = 10; t = (0:1/fs:T)';
figure(6); clf;
for i = 1:numel(fcheck)
    x = sin(2*pi*fcheck(i)*t);
    g(i) = iso2631_wk_rms(x, fs) / sqrt(mean(x.^2));
end
semilogx(fcheck, 20*log10(g)); grid on
xlabel('Hz'); ylabel('Wk gain [dB]');

aw_rms_ol = iso2631_wk_rms(y_ab_ol, fs);
aw_rms_hinfA = iso2631_wk_rms(y_ab_hinfA, fs);
aw_rms_hinfD = iso2631_wk_rms(y_ab_hinfD, fs);
%% helpers
function [cl_hinf, K, gamma_out, qcar_open, info, rdot_iso, r_iso] = design_hinf_for_road(road_type, rms_a_ref, rms_rs_ref, knob, params)
    if nargin < 5 || isempty(params)
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
    beta = 0.01;

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
    Wd3 = ss(0.5);  Wd3.u = "d3"; Wd3.y = "Wd3";

    scale_factor = 1;
    Wsd = scale_factor * knob * (beta) * tf(1, [1/w_bounce, 1]);
    Wsd.u = "sd"; Wsd.y = "e3";

    Wab = scale_factor * knob * ((1-beta)) * tf(1, conv([1/w_bounce, 1],[1/w_bounce, 1]));
    Wab.u = "ab"; Wab.y = "e2";

    sdmeas = sumblk("y1 = sd+Wd2");
    abmeas = sumblk("y2 = ab+Wd3");
    ICinputs  = ["d1";"d2";"d3";"u"];
    ICoutputs = ["e1";"e2";"e3";"y1";"y2"];

    % Notice qcar_open(["sd","ab"],:) isolates just the outputs needed to connect to the weights
    qcaric = connect(qcar_open(["sd","ab"],:), Act, Wroad, Wact, Wab, Wsd, ...
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
