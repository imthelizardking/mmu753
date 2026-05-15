%% Parameters
V = 20;             % Vehicle speed (m/sec)
fs = 1000;          % Sampling frequency (Hz)
T = 5;              % Simulation time (seconds)
t = 0:1/fs:T;       % Time vector
sigma_w = 0.1;      % Intensity of white noise
rho = 0.45;         % Road roughness constant (depends on road type)

%% 1. Generate White Noise Input w(t)
w = sigma_w * randn(size(t));

%% 2. Define the Speed Dependent Filter G(s)
% The transfer function for a common road model is:
% G(s) = V / (s + rho*V)
num = [V];
den = [1, rho*V];
sys = tf(num, den);

%% 3. Generate Road Displacement z_o(t)
% Filter the white noise to get "Colored Noise"
z_o = lsim(sys, w, t);

%% Plotting
figure;
subplot(2,1,1);
plot(t, w);
title('White Noise Input (Raw Randomness)');
ylabel('Intensity');

subplot(2,1,2);
plot(t, z_o, 'r', 'LineWidth', 1.5);
title(['Colored Noise Road Profile (Displacement) at V = ', num2str(V), ' m/s']);
xlabel('Time (sec)');
ylabel('Displacement (m)');
grid on;