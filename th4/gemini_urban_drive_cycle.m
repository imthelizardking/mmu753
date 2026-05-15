% Custom_EPA_Urban_Cycle.m
% Manual generation of an Urban Drive Cycle without toolboxes
% clear all, close all

% Time vector from 0 to 1400 seconds
t = 0:1:1400;

% Initializing speed vector (mph)
v = zeros(size(t));

% Creating "representative" urban segments to match image_684be1.png
% Segment 1: Initial stop-and-go
v(1:100) = 20 * (1 - cos(2*pi*(1:100)/50)); 
% Segment 2: Higher speed "hill" (The 55-60 mph peak)
v(200:350) = 55 * exp(-(( (200:350) - 260).^2) / (2 * 40^2)); 
% Segment 3: Series of smaller city blocks (20-35 mph)
v(400:1300) = 15 + 15*sin((400:1300)/15) + 5*randn(size(400:1300));
v(v < 0) = 0; % Ensure no negative speeds from the random noise

% Apply a moving average filter to smooth the "custom" noise
v = movmean(v, 10);

% --- Plotting to match image_684be1.png style ---
figure(1)
plot(t, v, 'Color', [0.8 0.8 0.8], 'LineWidth', 1) % Light gray line

% Formatting
xlabel('Time (Sec)')
ylabel('Speed (mph)')
title('EPA Urban cycle')

% Set axis limits to match your reference exactly
axis([0 1500 0 60])

% Aesthetic tweaks for the "Control Systems" look
grid on
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.5)