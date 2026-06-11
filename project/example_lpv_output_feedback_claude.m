%% Output-feedback LPV example: blending two observer-based controllers
% Shows that LPV does NOT require state feedback. Each controller is a
% dynamic output-feedback compensator (LQG-style):
%
%   observer:  xhat_dot = A*xhat + B*u + L*(y - C*xhat)
%   control:   u        = -K*xhat
%
% The controller STATE is xhat (the plant-state estimate). Because both
% designs estimate the SAME physical states through the SAME plant model,
% xhat has a consistent meaning, so blending K(rho) and L(rho) is valid.
%
% Requires the Control System Toolbox.

clear; clc;

%% Plant (same lightly damped oscillator)
A = [ 0   1;
     -1  -0.2];
B = [0; 1];
C = [1 0];
D = 0;
n = size(A,1);

%% Design 1 ("gentle"): feedback gain K1 and observer gain L1
K1 = lqr(A, B, diag([1 1]), 1);
L1 = lqr(A', C', diag([1 1]), 1)';        % observer via duality

%% Design 2 ("aggressive"): faster control and faster observer
K2 = lqr(A, B, diag([100 1]), 1);
L2 = lqr(A', C', diag([1 1]), 0.01)';     % small meas. weight -> fast observer

%% Scheduled gains and the full closed-loop matrix (plant + observer states)
Kr = @(rho) (1-rho)*K1 + rho*K2;
Lr = @(rho) (1-rho)*L1 + rho*L2;

% State vector is [x ; xhat]. Derivation:
%   x_dot    = A*x - B*K*xhat
%   xhat_dot = L*C*x + (A - B*K - L*C)*xhat
Acl = @(rho) [ A,            -B*Kr(rho);
               Lr(rho)*C,     A - B*Kr(rho) - Lr(rho)*C ];
Ccl = [C, zeros(1,n)];                     % output y = C*x

x0 = [1; 0; 0; 0];      % plant displaced, estimate starts at zero
t  = 0:0.01:15;

%% 1) Frozen cases incl. the average (rho = 0.5)
y1   = initial(ss(Acl(0),   zeros(2*n,1), Ccl, 0), x0, t);
y2   = initial(ss(Acl(1),   zeros(2*n,1), Ccl, 0), x0, t);
yavg = initial(ss(Acl(0.5), zeros(2*n,1), Ccl, 0), x0, t);

%% 2) True LPV: rho varies in time
rho_fun = @(tt) 0.5*(1 + sin(2*pi*tt/15));
dt = 1e-3; tv = 0:dt:15;
x = x0; y_lpv = zeros(size(tv)); rho_log = zeros(size(tv));
for k = 1:numel(tv)
    rho        = rho_fun(tv(k));
    rho_log(k) = rho;
    y_lpv(k)   = Ccl*x;
    x          = x + dt*(Acl(rho)*x);      % forward Euler
end

%% Plot
figure;
subplot(2,1,1); hold on; grid on;
plot(t,  y1,   '--','DisplayName','Controller 1  (\rho=0)');
plot(t,  y2,   '--','DisplayName','Controller 2  (\rho=1)');
plot(t,  yavg, '-', 'LineWidth',1.5,'DisplayName','Average  (\rho=0.5)');
plot(tv, y_lpv,'-', 'LineWidth',1.5,'DisplayName','LPV  (\rho varies)');
ylabel('output y'); legend('Location','northeast');
title('Blending two observer-based (output-feedback) controllers');

subplot(2,1,2); grid on;
plot(tv, rho_log,'LineWidth',1.5);
xlabel('time [s]'); ylabel('\rho(t)'); ylim([-0.05 1.05]);
title('Scheduling parameter');