clf; clear all; clc;
%% basic vibration isolator
% just sprung mass (M) connected to the ground with active suspension which
% gives force input u.
% states: x1: velocity of body (y_dot)
%         x2: rattle space (y-z)
% inputs: u:  active suspension force
%         w:  velocity of road surface
% notes: no connection to the body velocity, bad example
%%
M = 250; % body mass
A = [0, 0;...
     1, 0];
B = [1/M;...
     0];
L = [0;...
     -1];
C = eye(size(A));
D = 0;
% create systems
sys_passive = ss(A,L,C,D);
rho_1 = 0; % penalize state 1, body velocity
rho_2 = 10000; % penalize state 2, rattle space
Q = diag([rho_1, rho_2]);
R = .001; % minimize control input
[K, ~, ~] = lqr(A, B, Q, R);
sys_activelqr = ss(A-B*K, L, C, D);
%% plot time
t = 0:0.1:1; t = reshape(t,[max(size(t)),1]);
% u = zeros(size(t,1),1);
z = (rand(size(t,1),1)-1)*2+1;
% w = [diff(z);0];
w = 0.1*ones(size(t,1),1);
[y_passive,t_out_passive,x] = lsim(sys_passive,w,t);
[y_lqr,t_out_lqr,x] = lsim(sys_activelqr,w,t);
% Plotting
subplot(2,1,1);
plot(t_out_passive, y_passive(:,1), 'b', t_out_lqr, y_lqr(:,1), 'r', 'LineWidth', 1.5);
ylabel('Body Velocity (m/s)');
legend('passive','lqr');
grid on;

subplot(2,1,2);
plot(t_out_passive, y_passive(:,2), 'b', t_out_lqr, y_lqr(:,2), 'r', 'LineWidth', 1.5);
ylabel('Rattle Space (m)');
xlabel('Time (s)');
legend('passive','lqr');
grid on;
title('Active Suspension Response');
%% plot freq
bodemag(sys_passive);
hold on;
bodemag(sys_activelqr);
legend('passive', 'active');
grid minor;