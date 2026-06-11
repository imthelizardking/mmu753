%% LPV + H-infinity active suspension via gain-scheduled synthesis (hinfgs)
% ------------------------------------------------------------------------
% Instead of synthesising N independent H-inf controllers and blending them
% (which gave unstable blends), we solve ONE gain-scheduled H-inf problem
% over the whole beta range. hinfgs returns a polytopic controller whose
% convex interpolation is certified stable for every admissible beta(t) by a
% single quadratic Lyapunov function.
%
% Requires: Robust Control Toolbox (hinfgs, psys, pvec, ltisys, ltiss,
%           psinfo, polydec) + your helpers roadprofile_fun,
%           get_speed_for_road, get_vehicle_params on the path.
%
% THREE LEGACY-API SPOTS TO SANITY-CHECK against `doc` if anything errors:
%   (1) hinfgs(pdP,[NMEAS NCON]) -> here [2 1]  (2 measurements, 1 control)
%   (2) the affine "derivative" system s1 must carry E = 0  -> ltisys(...,Z)
%   (3) vertex extraction via psinfo/ltiss and weight order from polydec
% ------------------------------------------------------------------------

%% ---- user knobs -------------------------------------------------------
params        = 'nom';      % vehicle parameter set (get_vehicle_params)
road_baseline = 'Type C';   % road used to shape the synthesis weights
road_sim      = 'Type C';   % road used for the time simulation
knob          = 1.0;        % single fixed weight scale (push down for lower gamma)

%% ---- physical plant (displacement-input) + actuator -------------------
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

%% ---- build the generalised plant at a NOMINAL beta, then go affine ----
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

s0 = ltisys(Am, Bm, C0, D0);                         % E0 = I  (default)
s1 = ltisys(zeros(n), zeros(size(Bm)), C1, D1, zeros(n));  % E1 = 0
pv  = pvec('box', [0 1]);
pdP = psys(pv, [s0 s1]);

%% ---- gain-scheduled H-infinity synthesis ------------------------------
[gopt, pdK] = hinfgs(pdP, [2 1]);     % 2 measurements, 1 control
fprintf('hinfgs guaranteed gamma over beta in [0,1]: %.4f\n', gopt);

% pull out the vertex controllers (one per box vertex: beta=0 and beta=1)
[~, nv, nsK] = psinfo(pdK);
Av = cell(1,nv); Bv = cell(1,nv); Cv = cell(1,nv); Dv = cell(1,nv);
for j = 1:nv
    [Av{j},Bv{j},Cv{j},Dv{j}] = ltiss(psinfo(pdK,'sys',j));
end
fprintf('controller order = %d, vertices = %d\n', nsK, nv);

%% ---- stability check across beta (this is what failed before) ---------
fprintf('\n--- frozen closed-loop stability sweep ---\n');
for b = 0:0.1:1
    Acl = closed_loop_A(b, pv, Av,Bv,Cv,Dv, Ap,Bp,Cp,Dp, Aa,Ba,Ca);
    fprintf('beta=%.1f : stable=%d , max Re(pole)=%8.3f\n', ...
            b, all(real(eig(Acl))<0), max(real(eig(Acl))));
end

%% ---- road for the time simulation -------------------------------------
u0 = get_speed_for_road(road_sim);
[~, t_road, hsum, ~] = roadprofile_fun(road_sim, u0);
r_road = hsum(1:numel(t_road));            % road displacement [m]

%% ---- LPV time simulation: beta(t) actually varies ---------------------
nx = 4 + size(Aa,1) + nsK;                 % plant + actuator + controller
X0 = zeros(nx,1);
odef = @(t,X) lpv_rhs(t, X, pv, Av,Bv,Cv,Dv, ...
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

%% ---- plots ------------------------------------------------------------
figure; clf;
subplot(4,1,1); plot(tt,ab,'k'); grid on; ylabel('a_b [m/s^2]');
title('LPV+H_\infty active suspension, scheduled on \beta(t)');
subplot(4,1,2); plot(tt,sd*100,'k'); grid on; ylabel('sd [cm]');
subplot(4,1,3); plot(tt,td*100,'k'); grid on; ylabel('td [cm]');
subplot(4,1,4); plot(tt,bb,'r','LineWidth',1.2); grid on;
ylabel('\beta(t)'); xlabel('time [s]'); ylim([-0.05 1.05]);

%% ======================== local functions =============================
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

function [Ak,Bk,Ck,Dk] = Kof(beta, pv, Av,Bv,Cv,Dv)
% Interpolate vertex controllers with polytopic (barycentric) weights.
    c = polydec(pv, beta);            % nv x 1 convex weights for this beta
    Ak = zeros(size(Av{1})); Bk = zeros(size(Bv{1}));
    Ck = zeros(size(Cv{1})); Dk = zeros(size(Dv{1}));
    for j = 1:numel(Av)
        Ak = Ak + c(j)*Av{j};  Bk = Bk + c(j)*Bv{j};
        Ck = Ck + c(j)*Cv{j};  Dk = Dk + c(j)*Dv{j};
    end
end

function Acl = closed_loop_A(beta, pv, Av,Bv,Cv,Dv, Ap,Bp,Cp,Dp, Aa,Ba,Ca)
% Frozen closed-loop A for [xp; xa; xk] with road set to zero.
    [Ak,Bk,Ck,Dk] = Kof(beta, pv, Av,Bv,Cv,Dv);
    Cym = [Cp(2,:); Cp(3,:)];          % measurements [sd; ab] from plant states
    Dyf = [0; Dp(3,2)*Ca];             % ab feedthrough from fs (= Ca*xa)
    Bf  = Bp(:,2)*Ca;                  % fs path into the plant
    Acl = [ Ap,            Bf,                 zeros(4,numel(Ck))*0 + 0*Ck' ; %#ok
            Ba*Dk*Cym,     Aa + Ba*Dk*Dyf,     Ba*Ck;
            Bk*Cym,        Bk*Dyf,             Ak ];
    Acl(1:4, 4+size(Aa,1)+1:end) = 0;  % plant has no direct xk coupling
end

function dX = lpv_rhs(t, X, pv, Av,Bv,Cv,Dv, Ap,Bp,Cp,Dp, Aa,Ba,Ca, t_road, r_road)
    na = size(Aa,1);  nk = size(Av{1},1);
    xp = X(1:4);  xa = X(5:4+na);  xk = X(4+na+1:4+na+nk);

    r  = interp1(t_road, r_road, t, 'linear', 0);
    fs = Ca*xa;

    sd = Cp(2,:)*xp;
    ab = Cp(3,:)*xp + Dp(3,2)*fs;
    y  = [sd; ab];

    [Ak,Bk,Ck,Dk] = Kof(beta_of_t(t), pv, Av,Bv,Cv,Dv);
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