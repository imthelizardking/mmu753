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
t = 0:0.01:10;
r = zeros(size(t));
ramp = 0:0.01:2;
idx_start = 5;
r(idx_start:idx_start+size(ramp,2)-1) = ramp;
% r = rand(size(t))/10;
f_c = 0 * ones(size(t));
u = [r', f_c'];
[y,tOut,x] = lsim(sys, u, t);
plot(t,x(:,1));
hold on
plot(t,r);
legend
grid minor