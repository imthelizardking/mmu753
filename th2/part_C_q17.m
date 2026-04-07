clf; clear all; clc;
%% PART C (17)
% full car modeled based on jazar
% control strategy based on modeling each cırner as quarter car,
% independently
% states (14x1): (x, theta, phi, x1, x2, x3, x4 and their derivatives
%%
FOUND_full_car;
fullcar = sys;
% extend with control input matrix
B_u = [ 1/m,        1/m,        1/m,        1/m;       % Heave
           -a1/Iy,    -a1/Iy,     a2/Iy,     a2/Iy;    % Pitch
            (w/2)/Ix, -(w/2)/Ix, (w/2)/Ix, -(w/2)/Ix;% Roll
           -1/mf,      0,          0,          0;         % Wheel FL
            0,         -1/mf,      0,          0;         % Wheel FR
            0,          0,         -1/mr,      0;         % Wheel RL
            0,          0,          0,         -1/mr ];   % Wheel RR
B_u = [zeros(7,4); B_u];
% fullcar.StateName = {'heave velocity'; 'pitch velocity'; 'roll velocity';...
%     'tire1 velocity'; 'tire2 velocity'; 'tire3 velocity'; 'tire4 velocity';...
%     'heave'; 'pitch'; 'roll';...
%     'tire1 deflection'; 'tire2 deflection'; 'tire3 deflection'; 'tire4 deflection'};
% fullcar.InputName = {'r1'; 'r2'; 'r3'; 'r4'; 'f1'; 'f2'; 'f3'; 'f4';};
% fullcar.OutputName = {'heave velocity'; 'pitch velocity'; 'roll velocity';...
%     'tire1 velocity'; 'tire2 velocity'; 'tire3 velocity'; 'tire4 velocity';...
%     'heave'; 'pitch'; 'roll';...
%     'tire1 deflection'; 'tire2 deflection'; 'tire3 deflection'; 'tire4 deflection'};  
%% quarter car modelling:

k_vec = [kf; kf; kr; kr];
kt_vec = [ktf; ktf; ktr; ktr];
c_vec = [cf; cf; cr; cr];
m_vec = [mf; mf; mr; mr];
bt = 0;
rho1 = 100000; % body acc.
rho2 = 0; % rattlespace
rho3 = 0; % rattlespace
rho4 = 0; % rattlespace
Q = diag([rho1, rho2, rho3, rho4]);
R = .001;
for i=1:4
    Aq(:,:,i) = [0, 1, 0, -1;...
     -k_vec(i)/m, -c_vec(i)/m, 0, c_vec(i)/m;...
     0, 0, 0, 1;...
     k_vec(i)/m_vec(i), c_vec(i)/m_vec(i), -kt_vec(i)/m_vec(i), -(c_vec(i)+bt)/m_vec(i)];
    Bq(:,:,i) = [0;...
        1/m;...
        0;...
        -1/m_vec(i)];
    % lqr design
    [K_(:,:,i), ~, ~] = lqr(Aq(:,:,i), Bq(:,:,i), Q, R);
end
Cq = eye(size(A));
Dq = 0;
L = [0;...
     0;...
     -1;...
     0];
K_local = [ K_(:,:,1),   zeros(1,4), zeros(1,4), zeros(1,4);
            zeros(1,4), K_(:,:,2),   zeros(1,4), zeros(1,4);
            zeros(1,4), zeros(1,4), K_(:,:,3),   zeros(1,4);
            zeros(1,4), zeros(1,4), zeros(1,4), K_(:,:,4)];
T_sub = [ 1, -a1,  w/2,  0, 0, 0, 0;  % zs_fl
          0,  0,    0,    1, 0, 0, 0;  % zu_fl (Wheel 1)
          1, -a1, -w/2,  0, 0, 0, 0;  % zs_fr
          0,  0,    0,    0, 1, 0, 0;  % zu_fr (Wheel 2)
          1,  a2,  w/2,  0, 0, 0, 0;  % zs_rl
          0,  0,    0,    0, 0, 1, 0;  % zu_rl (Wheel 3)
          1,  a2, -w/2,  0, 0, 0, 0;  % zs_rr
          0,  0,    0,    0, 0, 0, 1]; % zu_rr (Wheel 4)
T = blkdiag(T_sub, T_sub);
K_full = K_local * T; % This is 4 x 14
fullcar_closedloop = ss(A-B_u*K_full,B,C,D);