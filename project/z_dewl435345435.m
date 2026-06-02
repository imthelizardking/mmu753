clf; clear all; clc;
%%
gain = 1; freq = 77;
signal = gain * tf(1, [1/freq, 1]);
signal_2 = gain * tf(1, conv([1/freq, 1], [1/freq, 1]));
bodemag(signal); hold on;
bodemag(signal_2);
legend('1st order', 'roll-of');
grid minor;
