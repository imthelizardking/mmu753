clc; clear; close all;

%% ABS Algorithm Thresholds
%Algo 1
S1 = 0.02; S2 = 0.05; %slip threshold
% S1 = 0.02; S2 = 0.15; %slip thresholdd

%Algo 2
A1 = -40; A2 = 2; %acceleration threshold

%%
g = 9.81;   % Gravitational stiffness
rw1 = 0.4; rw2 = rw1; rw3 = rw1; rw4 = rw1;    %Wheel Radii
rs1 = rw1; rs2 = rw1; rs3 = rw1; rs4 = rw1;     %Height of suspension lower hardpoint above ground

Bh = 5;  Ch = 2*1.2;   Dh = 0.3; Eh = 1;   %Snow Magic Formula Parameters
Bs = 10; Cs = 1.9*1.2; Ds = 1;   Es = 0.97; %Dry Tarmac Magic Formula Parameters

Xtire = 20; ntire = 10; %Tire force parameters for combined slip

intVEL = 20; %intVEL = 1;   % Initial Velocities
Vox = intVEL; Voy = 0;

% Vehicle Inertias
Is = [0.3e6 0 0;
      0 1e6 0;
      0 0 0.8e6];

Izz = Is(3,3); % Yaw inertia

Rw = rw1; % Wheel raidus

hs = 0.20; %Height of COG above suspension hardpoint
Jw = 10; % Tire inertia
Ms = 7000; % Vehicle Mass
a = 1.5; % Front axel COG distance
b = 3; % Rear acle COG 
T = 2.5; % Track Width

cf = 7.5e5; cr = 7.5e5; % Cornering Stinfess

ARV = 4; CDV = 0.7; pr = 1.225; % Air resistance parameters

%% Pneumatic Data - Pneumatic and Brake Parameters
P_s = 8e5; Patm = 101356;
Ab = 0.01;
wang = deg2rad(12);
WBF = 4;
wr = 0.4;
wp = 1/(2*(tan(wang/2)));
nm = 1;
Po = 50e3;
fa = 1; ff = 1;

%% yco
S_ref = 1.1;
Kbf = -18;
Kbr = -4;
Ts = 0.0379; % max load independent static friction Nm
R = 0.05; % resistance, ohm
L = 56*10^(-6); % inductance, henry
Kt = 0.0697; % motor torque constant Nm/a
Kb = Kt * 2/3; % back emf constant Nm/A
J = 0.291*10^(-3); % effective inertia kgm^2
N = 0.0263; % total gear reduction mm/rad
