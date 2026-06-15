clear all; clc;
%% params:
% TRIKE_PARAMS  Parameters for the Q2 delta three-wheeler plant model.
%
%   Configuration (Yusuf):
%     - Delta layout: 1 wheel front, 2 wheels rear
%     - Front single wheel is DRIVEN and STEERED
%     - Rear two wheels are BRAKE-ONLY  -> ESC = differential rear braking
%
%   Defines every constant used by the 9-state model
%       x = [u v r omega_f omega_rl omega_rr X Y psi]
%   with inputs
%       in = [delta T_d T_b_f T_b_rl T_b_rr]
%
%   Units are SI throughout (m, kg, s, N, rad).  Values marked [TUNE] are
%   reasonable placeholders for a light passenger trike; refine to your case.

%% -------------------------------------------------------------------------
%  Environment
% --------------------------------------------------------------------------
g    = 9.81;          % gravitational acceleration            [m/s^2]

%% -------------------------------------------------------------------------
%  Mass and inertia (sprung + unsprung lumped; planar rigid body)
% --------------------------------------------------------------------------
m    = 500;           % total vehicle mass            [kg]   [TUNE]
Iz   = 550;           % yaw moment of inertia about CG [kg*m^2] [TUNE]
h    = 0.50;          % CG height (load transfer / rollover) [m] [TUNE]

%% -------------------------------------------------------------------------
%  Geometry
% --------------------------------------------------------------------------
lf   = 0.85;          % CG -> front axle distance      [m]   [TUNE]
lr   = 0.75;          % CG -> rear axle distance       [m]   [TUNE]
L    = lf + lr;       % wheelbase                      [m]
tr   = 0.60;          % HALF rear track (rear wheel lateral offset) [m] [TUNE]
%                       => full rear track = 2*tr

%% -------------------------------------------------------------------------
%  Wheel / rotational dynamics  (Iw*omega_dot = T - Re*Fx - T_b)
% --------------------------------------------------------------------------
Iw   = 1.0;           % wheel spin inertia (per wheel) [kg*m^2] [TUNE]
Re   = 0.28;          % effective rolling radius       [m]   [TUNE]

%% -------------------------------------------------------------------------
%  Friction
% --------------------------------------------------------------------------
mu       = 0.90;      % nominal road friction coefficient    [-]
mu_low   = 0.30;      % low-mu value (split-mu / icy patch)   [-]  (Q3)

%% -------------------------------------------------------------------------
%  Tire model -- LINEAR (controller design / observer Jacobians)
%    F_y = C_alpha * alpha ,   F_x = C_s * s        (per tire)
% --------------------------------------------------------------------------
Calf  = 30000;        % front cornering stiffness, per tire [N/rad] [TUNE]
Calr  = 28000;        % rear  cornering stiffness, per tire [N/rad] [TUNE]
Cs    = 50000;        % longitudinal slip stiffness, per tire [N]   [TUNE]

%% -------------------------------------------------------------------------
%  Tire model -- PACEJKA magic formula (nonlinear "truth" plant, Q5)
%    F = D*sin(C*atan(B*x - E*(B*x - atan(B*x))))   with  D = mu*Fz
% --------------------------------------------------------------------------
Bx = 10.0;  Cx = 1.65;  Ex = 0.97;   % longitudinal [TUNE]
By = 8.0;   Cy = 1.30;  Ey = 0.97;   % lateral      [TUNE]

%% -------------------------------------------------------------------------
%  Static normal loads (delta geometry: front single, rear pair)
%    Used directly by the friction circle and as nominal Fz.
% --------------------------------------------------------------------------
Fzf  = m * g * lr / L;          % front wheel           [N]
Fzr  = m * g * lf / (2*L);      % each rear wheel       [N]
%   (longitudinal/lateral transfer added dynamically in f(x,u) if used)

%% -------------------------------------------------------------------------
%  Aerodynamics (optional drag; set Cd=0 to disable)
% --------------------------------------------------------------------------
rho  = 1.225;         % air density                    [kg/m^3]
Cd   = 0.6;           % drag coefficient               [-]   [TUNE]
Af   = 1.8;           % frontal area                   [m^2] [TUNE]

%% -------------------------------------------------------------------------
%  Actuator limits  (enforced in the low-level allocator, Q4)
% --------------------------------------------------------------------------
Td_max   = 400;       % max front drive torque         [N*m] [TUNE]
Tb_max   = 1200;      % max brake torque per wheel     [N*m] [TUNE]
delta_max      = deg2rad(30);    % max steer angle      [rad]
delta_rate_max = deg2rad(400);   % max steer rate       [rad/s]

%% -------------------------------------------------------------------------
%  Numerical / simulation
% --------------------------------------------------------------------------
Ts    = 1e-3;         % fixed-step solver / sample time [s]  (matches papers)
eps_v = 0.5;          % low-speed guard for slip ratio  [m/s]
%                       prevents the v->0 singularity at end of hard stops

%% -------------------------------------------------------------------------
%  Initial conditions (override per scenario)
% --------------------------------------------------------------------------
u0    = 25;           % initial longitudinal speed (90 km/h) [m/s]
x0    = [u0; 0; 0; ...                 % u, v, r
         u0/Re; u0/Re; u0/Re; ...      % omega_f, omega_rl, omega_rr (free-rolling)
         0; 0; 0];                     % X, Y, psi (global pose)