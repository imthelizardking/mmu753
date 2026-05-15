clf; clear all; clc;

%% 1. Vehicle Parameters
m = 1250;
ro = 1.225;
A_front = 1.2;
Cd = 0.5;
f = 0.015;
R = 3;         % Final drive ratio
I_e = 0.15;    % Engine inertia
r = 0.3;       % Wheel radius
I_w = 1.0;     % Inertia per wheel
u_w = 2;       % Wind speed
g = 9.81;
theta_0 = 0;   % Road angle in radians

%% 2. Operating Points and System Setup
u_0s = [20; 30; 37; 48; 73];
I_w_total = 4 * I_w;
s = tf('s');

% We will loop to visualize the Bodes, then tune for index i=3 (37 m/s)
for i = 1:size(u_0s,1)
    u_curr = u_0s(i);
    
    % Gear Logic
    if u_curr <= 25, G = 2.2;
    elseif u_curr <= 35, G = 1.5;
    elseif u_curr <= 45, G = 1.1;
    else, G = 0.85;
    end
    
    % Equivalent Mass Calculation
    m_eqv = m + I_w_total/r^2 + I_e*(G*R)^2/r^2;
    
    % Linearization Parameters
    % gamma is the 'damping' coefficient from air drag derivative
    gamma = ro * A_front * Cd * (u_curr + u_w);
    
    % Transfer Function: G(s) = (1/gamma) / ( (m_eqv/gamma)s + 1 )
    tau_val = m_eqv / gamma;
    K_val = 1 / gamma;
    
    sys_lin(:,:,i) = K_val / (tau_val * s + 1);
    
    figure(1);
    bode(sys_lin(:,:,i)); hold on;
end
title('Bode Plots for Different Operating Speeds');
legend(string(u_0s));

%% 3. Controller Design for u_0 = 37 m/s (i=3)
idx = 3;
u_target = u_0s(idx);

% Re-calculate specific parameters for this point
if u_target <= 25, G = 2.2;
elseif u_target <= 35, G = 1.5;
elseif u_target <= 45, G = 1.1;
else, G = 0.85;
end

m_eqv = m + I_w_total/r^2 + I_e*(G*R)^2/r^2;
gamma = ro * A_front * Cd * (u_target + u_w);

% --- PHYSICAL STATE SPACE ---
% State x1 = velocity deviation (u')
Ap = -gamma / m_eqv;
Bp = 1 / m_eqv;
Cp = 1;

% Augmented State: [u'; integral_of_error]
% Error is defined as (u_set - u), so x2_dot = -Cp*x1
A_aug = [Ap,  0; 
        -Cp,  0]; 
B_aug = [Bp; 
         0];

%% 4. Pole Placement Requirements
Mp = 4;    % 4% overshoot
ts = 10;   % 10 second settling time

zeta = -log(Mp/100) / sqrt(pi^2 + (log(Mp/100))^2);
omega_n = 4 / (zeta * ts);

% Calculate complex conjugate poles
p1 = -zeta*omega_n + 1i*omega_n*sqrt(1-zeta^2);
p2 = conj(p1);
p = [p1, p2];

% Calculate Gains
K_gains = place(A_aug, B_aug, p);

% Mapping to Simulink PI Controller
% F_control = Kp*err + Ki*integral(err)
Kp = K_gains(1);
Ki = -K_gains(2); % Negative because of the -Cp in the A_aug matrix

fprintf('Design Results for u0 = %d m/s:\n', u_target);
fprintf('Kp = %.2f\n', Kp);
fprintf('Ki = %.2f\n', Ki);
fprintf('Damping Ratio (zeta) = %.2f\n', zeta);