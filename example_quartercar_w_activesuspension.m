close all; clear all; clc;
% Physical parameters
mb = 300;    % kg
mw = 60;     % kg
bs = 1000;   % N/m/s
ks = 16000 ; % N/m
kt = 190000; % N/m

% State matrices
A = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
      0 0 0 1; [ks bs -ks-kt -bs]/mw];
B = [ 0 0; 0 1e3/mb ; 0 0 ; [kt -1e3]/mw];
C = [1 0 0 0; 1 0 -1 0; A(2,:)];
D = [0 0; 0 0; B(2,:)];
sys = ss(A,B,C,D);
ts = 0.01;
t = 0:ts:10;
r = zeros(size(t));
ramp = 0:ts:2;
idx_start = 5;
r(idx_start:idx_start+size(ramp,2)-1) = ramp;
f_c = 0 * ones(size(t));
u = [r', f_c'];
[y,tOut,x] = lsim(sys, u, t);
plot(t,x(:,1));
hold on
plot(t,r);
legend
grid minor
PID = pid(0,0,0);
[b,a] = ss2tf(A,B,C,D,2);
sys_tf = tf(b,a);
sys_cl = feedback(sys_tf,PID);
[y,tOut,x] = lsim(sys_cl, u, t);
figure
plot(t,x(:,1));
hold on
plot(t,r);
legend
grid minor

%err_pre = 0; err = 0; err_i = 0;
% Kp = 0; Ki = 0; Kd = 0;
% x = [0;0;0;0];
% x_save = zeros(max(size(t))+1,4);
% r_save = zeros(max(size(t)),1);
% x_save(1,:) = x; idx = 1;
% for t_=t
%     err_p = err;
%     err_i = err_i + err_p;
%     err_d = (err-err_pre)/ts;
%     u_t = Kp*err_p + Ki*err_i + Kd*err_d;
%     r_t = r(idx);
%     r_save(idx) = r_t;
%     u_save(idx) = u_t;
%     x_dot = A*x+B*[r_t; u_t];
%     y = C*x+D*[r_t; u_t];
%     x = x+x_dot*ts;
%     err_pre = err;
%     idx = idx + 1;
%     x_save(idx,:) = x;
%     err = x_save(idx,1) - x_save(idx-1,1);
% end
% figure
% plot(t,x_save(1:end-1,1),'DisplayName', 'body acc');
% hold on
% plot(t,r_save,'DisplayName', 'road profile');
% % hold on
% % plot(t,u_save);
% legend
% grid minor