% ... (Keep your parameters m, ro, A, Cd, etc.)

% 1. Get the Open Loop Plant (1st Order)
i = 3; % Selecting u_0 = 37
[num, den] = tfdata(sys_lin(:,:,i), 'v');
[Ap, Bp, Cp, Dp] = tf2ss(num, den);

% 2. Augment the system with an integrator (to handle the steady-state error)
% New state vector: [velocity; integral_of_error]
A_aug = [Ap, 0; -Cp, 0]; 
B_aug = [Bp; 0];

% 3. Calculate Desired Poles (Your math here was mostly correct)
zeta = -log(Mp/100) / sqrt(pi^2 + (log(Mp/100))^2);
sigma = 4 / ts;
wd = sigma * sqrt(1 - zeta^2) / zeta;
p = [-sigma + 1i*wd, -sigma - 1i*wd];

% 4. Pole Placement on Augmented System
K_gain = place(A_aug, B_aug, p);

% 5. Create the Closed Loop System
% The feedback is u = -K1*v - K2*integral(error)
A_cl = A_aug - B_aug * K_gain;
B_cl = [0; 1]; % Reference input enters through the integrator
C_cl = [Cp, 0];
D_cl = 0;

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% 6. Plot
t = 0:0.1:40;
step(sys_cl, t);
title('Corrected Cruise Control Step Response');