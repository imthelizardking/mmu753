clf; clear all; clc;
% Parameters
M = 300; % Body mass (kg)

% State-Space Matrices
% States: [y_dot; y-z]
% Inputs: [u (force); w (road velocity)]
A = [0, 0; 
     1, 0];

B = [1/M,  0; 
     0,   -1];

C = eye(2); % Output both states
D = zeros(2, 2);

sys = ss(A, B, C, D);

% Time vector
t = 0:0.01:5; 

% Define Inputs
% u: Let's assume a constant active force of 100N
% w: A "bump" represented by a short pulse in road velocity
u_input = 100 * zeros(size(t)); 
w_input = zeros(size(t));
w_input(t > 1 & t < 1.2) = 0.5; % Road velocity spike at 1s

U = [u_input; w_input]'; % Combine inputs (must be columns)

% Simulation
[y_out, t_out] = lsim(sys, U, t);

% Plotting
subplot(2,1,1);
plot(t_out, y_out(:,1), 'LineWidth', 1.5);
ylabel('Body Velocity (m/s)');
grid on;
title('Active Suspension Response');

subplot(2,1,2);
plot(t_out, y_out(:,2), 'r', 'LineWidth', 1.5);
ylabel('Rattle Space (m)');
xlabel('Time (s)');
grid on;