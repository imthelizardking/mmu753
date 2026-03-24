close all; clear all; clc;

Ts = 0.0379; % max load independent static friction Nm
R = 0.05; % resistance, ohm
L = 56*10^(-6); % inductance, henry
Kt = 0.0697; % motor torque constant Nm/a
Kb = Kt * 2/3; % back emf constant Nm/A
J = 0.291*10^(-3); % effective inertia kgm^2
N = 0.0263; % total gear reduction mm/rad