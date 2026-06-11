%% Basic LPV example: interpolating between two closed-loop systems
% Idea: two controllers -> two closed loops. Blending them with a
% scheduling parameter rho in [0,1] is a (polytopic) LPV system.
%   rho = 0   -> controller 1
%   rho = 1   -> controller 2
%   rho = 0.5 -> the AVERAGE you asked about
% Letting rho vary in time is the actual LPV behaviour.
%
% Requires the Control System Toolbox (lqr, ss, initial).

clear; clc;

%% Plant: a lightly damped 2nd-order oscillator (shared by both controllers)
A = [ 0   1;
     -1  -0.2];
B = [0; 1];
C = [1 0];
D = 0;

%% Two state-feedback controllers ("the two controllers you designed")
% Same plant, different LQR weights -> different behaviour.
K1 = lqr(A, B, diag([1   1]), 1);   % gentle
K2 = lqr(A, B, diag([100 1]), 1);   % aggressive

% Scheduled gain and closed-loop A-matrix as functions of rho.
% Because both controllers act on the SAME plant states, A - B*K(rho)
% is automatically in a common state basis, so blending is meaningful.
Kr  = @(rho) (1-rho)*K1 + rho*K2;
Acl = @(rho) A - B*Kr(rho);

x0 = [1; 0];          % initial condition (free response, no reference)
t  = 0:0.01:15;

%% 1) Frozen cases: the two controllers and their AVERAGE (rho = 0.5)
y1   = initial(ss(Acl(0),   B, C, D), x0, t);   % controller 1
y2   = initial(ss(Acl(1),   B, C, D), x0, t);   % controller 2
yavg = initial(ss(Acl(0.5), B, C, D), x0, t);   % average controller

%% 2) True LPV: let rho vary with time and integrate the time-varying system
rho_fun = @(tt) 0.5*(1 + sin(2*pi*tt/15));      % rho sweeps 0 -> 1 -> 0
dt = 1e-3;
tv = 0:dt:15;
x  = x0;
y_lpv   = zeros(size(tv));
rho_log = zeros(size(tv));
for k = 1:numel(tv)
    rho        = rho_fun(tv(k));
    rho_log(k) = rho;
    y_lpv(k)   = C*x;
    xdot       = Acl(rho)*x;     % control u = -K(rho)x is folded into Acl
    x          = x + dt*xdot;    % forward Euler (use ode45 for more accuracy)
end

%% Plot
figure;
subplot(2,1,1); hold on; grid on;
plot(t,  y1,   '--', 'DisplayName','Controller 1  (\rho=0)');
plot(t,  y2,   '--', 'DisplayName','Controller 2  (\rho=1)');
plot(t,  yavg, '-',  'LineWidth',1.5, 'DisplayName','Average  (\rho=0.5)');
plot(tv, y_lpv,'-',  'LineWidth',1.5, 'DisplayName','LPV  (\rho varies)');
ylabel('output y'); legend('Location','northeast');
title('Averaging two closed loops = midpoint of an LPV interpolation');

subplot(2,1,2); grid on;
plot(tv, rho_log, 'LineWidth',1.5);
xlabel('time [s]'); ylabel('\rho(t)'); ylim([-0.05 1.05]);
title('Scheduling parameter');