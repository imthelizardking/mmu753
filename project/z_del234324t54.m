%% Power Spectral Density (PSD) Tutorial Example
clear; clc; close all;

%% 1. Define Signal Parameters
Fs = 1000;                    % Sampling frequency (Hz)
t = 0:1/Fs:2-1/Fs;            % Time vector (2 seconds duration)
N = length(t);                % Number of samples

%% 2. Create the Signal (2 Sine Waves + Noise)
% We mix a 50 Hz wave, a 120 Hz wave, and random white noise.
f1 = 50;                      % First frequency component (Hz)
f2 = 120;                     % Second frequency component (Hz)

% Signal definition
clean_signal = 2*sin(2*pi*f1*t) + 3*cos(2*pi*f2*t);
noise = 2.5 * randn(size(t));  % White noise with variance
x = clean_signal + noise;     % Noisy signal

%% 3. Plot the Time-Domain Signal
figure('Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(t(1:200), x(1:200), 'LineWidth', 1.5); % Plot first 200 samples for clarity
title('Time Domain: Noisy Signal');
xlabel('Time (seconds)');
ylabel('Amplitude');
grid on;

%% 4. Calculate PSD using Welch's Method
% pwelch divides the signal into overlapping segments, computes the periodogram 
% for each, and averages them to reduce noise artifacts.
window = hanning(256);        % Window function to reduce spectral leakage
overlap = 128;                % 50% overlap between segments
nfft = 512;                   % FFT points (defines frequency resolution)

[pxx, f] = pwelch(x, window, overlap, nfft, Fs);

%% 5. Plot the Power Spectral Density (Frequency Domain)
subplot(2,1,2);
% We convert the raw power to decibels (dB/Hz) using 10*log10 for standard visualization
plot(f, 10*log10(pxx), 'r', 'LineWidth', 2);
title('Frequency Domain: Power Spectral Density (PSD)');
xlabel('Frequency (Hz)');
ylabel('Power/Frequency (dB/Hz)');
grid on;

% Highlight the identified frequencies
hold on;
plot(f1, 10*log10(pxx(find(f>=f1,1))), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
plot(f2, 10*log10(pxx(find(f>=f2,1))), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
text(f1+10, 10*log10(pxx(find(f>=f1,1))), ['\leftarrow ' num2str(f1) ' Hz Component']);
text(f2+10, 10*log10(pxx(find(f>=f2,1))), ['\leftarrow ' num2str(f2) ' Hz Component']);