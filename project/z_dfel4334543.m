clf; clear all; clc;
%%
A = 1; t = 0:0.1:10; w = 1;
w_hz = w/(2*pi)
signal1 = A*sin(w*t); % 10 rad/sec
signal2 = A*sin(w*1.5*t); % 15 rad/sec
plot(t, signal1)
%% psd:
window = hanning(16);        % Window function to reduce spectral leakage
overlap = 8;                % 50% overlap between segments
nfft = 32;                   % FFT points (defines frequency resolution)
Fs = 64;
x = signal1+signal2;
[pxx, f] = pwelch(x, window, overlap, nfft, Fs);
clf;
plot(f, 10*log10(pxx), 'r', 'LineWidth', 2);
title('Frequency Domain: Power Spectral Density (PSD)');
xlabel('Frequency (Hz)');
ylabel('Power/Frequency (dB/Hz)');
grid on;

% Highlight the identified frequencies
% hold on;
% plot(f1, 10*log10(pxx(find(f>=f1,1))), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
% plot(f2, 10*log10(pxx(find(f>=f2,1))), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
% text(f1+10, 10*log10(pxx(find(f>=f1,1))), ['\leftarrow ' num26str(f1) ' Hz Component']);
% text(f2+10, 10*log10(pxx(find(f>=f2,1))), ['\leftarrow ' num26str(f2) ' Hz Component']);