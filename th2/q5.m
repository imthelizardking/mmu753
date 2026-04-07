clf; clear all; clc;
%%
init_params;
A = [0, 1; -k/m, -c/m]; B = [0; 1/m]; L = [-1; c/m];
ro1 = 1; ro2 = 1;
Q = diag([ro1, ro2]);
R = 0.01;
[K, S, P] = lqr(A, B, Q, R);
disp('LQR Gains are:');
disp(K);