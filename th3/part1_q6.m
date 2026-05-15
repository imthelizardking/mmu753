clear all; clc;
part1_q4; % get model param.s
clf;
sys_openloop = sys_lin(:,:,i);
i = 3; % Selecting velocity u_0
u_0 = u_0s(i);
[num, den] = tfdata(sys_lin(:,:,i), 'v');
[Ap, Bp, Cp, Dp] = tf2ss(num, den);

% --- CORRECTED PHYSICAL MAPPING ---
% Physical parameters for selected u_0
gamma = ro * A * Cd * (u_0 + u_w); % Damping constant
% A = -gamma/m_eqv
% B = 1/m_eqv

% Define Augmented Matrices directly from physics
Ap_phys = -gamma / m_eqv;
Bp_phys = 1 / m_eqv;
Cp_phys = 1;

A_aug = [Ap_phys, 0; 
         -Cp_phys, 0];  % Error = u_set - u_actual
B_aug = [Bp_phys; 
         0];


Mp = 4; % maximum percent overshoot is allowed
ts = 10; % settling time allowed
zeta = -log(Mp/100)/...
    sqrt(pi^2+(log(Mp/100))^2);
sigma = 4/...
    (ts);
wd = sigma*sqrt(1-zeta^2)/(zeta);
p1 = -real(sigma)+real(wd)*1i;
p2 = conj(p1);
p = [p1, p2];
K_gains = place(A_aug, B_aug, p);
K_gains(2) = abs(K_gains(2));
Kp = K_gains(1); Ki = K_gains(2);
A_cl = A_aug - B_aug * K_gains;
B_cl = [0; 1]; % Reference input enters through the integrator
C_cl = [Cp_phys, 0];
D_cl = 0;
sys_closedloop_ss = ss(A_cl, B_aug, C_cl, D_cl);
%%
t=[0:0.5:20];
figure(1);
% step(sys_openloop,t);
% hold on;
step(sys_closedloop_ss,t);
% legend('OL','CL');
u_ramp = t;
[y, t_out, x] = lsim(sys_closedloop_ss, u_ramp, t);
figure(2)
plot(t, u_ramp, 'r--', t, y, 'b');
grid on;
legend('Ramp Input (Command)', 'System Response (Actual Speed)');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Cruise Control Ramp Response');
%%
u_set = 37;
F_x0_set = m*g*sin(theta_0) + f*m*g*cos(theta_0) + ...
    0.5*ro*A*Cd*(u_set+u_w)^2;