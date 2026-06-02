clf; clear all; clc;
w_low  = 2*pi*4;    % 25.13 rad/s
w_high = 2*pi*8;    % 50.27 rad/s
w_c    = sqrt(w_low * w_high);  % geometric center
Q      = w_c / (w_high - w_low);

% Bandpass weight: large gain between 4-8 Hz
% tf([numerator], [denominator]) — peaks at w_c
Wab_shape = tf([1, 0], ...
               [(1/w_c)^2, 1/(Q*w_c), 1]);

% Scale by normalization
gain_ab = 10;
Wab = gain_ab * Wab_shape;
Wab.u = 'body acceleration'; Wab.y = 'e2';

clf;
w_vec = logspace(-1, 3, 500);
bodemag(Wab, w_vec);
title('Wab shape — should peak between 4 and 8 Hz');
xline(w_low/(2*pi), 'r--', '4 Hz');
xline(w_high/(2*pi), 'r--', '8 Hz');
grid on;