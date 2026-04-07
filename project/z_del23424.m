% Simulation Setup
h = 0.05;             % Small time step for accuracy
t = 0:h:20;           % 20 seconds of flight
n = length(t);
y = zeros(4, n);

% Initial Conditions: [x=0, z=500m, vx=30m/s, vz=0]
y(:, 1) = [0; 500; 30; 0];

% RK4 Integration Loop
for i = 1:(n-1)
    k1 = glider_physics(t(i), y(:, i));
    k2 = glider_physics(t(i) + h/2, y(:, i) + h*k1/2);
    k3 = glider_physics(t(i) + h/2, y(:, i) + h*k2/2);
    k4 = glider_physics(t(i) + h, y(:, i) + h*k3);
    
    y(:, i+1) = y(:, i) + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    
    % Stop if we hit the ground
    if y(2, i+1) <= 0
        y(:, i+1:end) = []; % Clear remaining zeros
        break;
    end
end

% Plotting the Flight Path
plot(y(1,:), y(2,:), 'r', 'LineWidth', 2);
grid on;
xlabel('Horizontal Distance (m)');
ylabel('Altitude (m)');
title('Non-Linear Glider Path (RK4 Integration)');

function dydt = glider_physics(~, y)
    % y(1) = x (pos), y(2) = z (alt), y(3) = vx (vel), y(4) = vz (vel)
    
    g = 9.81;      % Gravity (m/s^2)
    rho = 1.225;   % Air density (kg/m^3)
    Cd = 0.05;     % Drag coefficient
    Cl = 0.5;      % Lift coefficient
    A = 2.0;       % Wing area (m^2)
    m = 50;        % Mass (kg)

    % Current speed
    v = sqrt(y(3)^2 + y(4)^2);
    
    % Dynamic Pressure
    q = 0.5 * rho * v^2 * A;
    
    % Forces
    Drag = q * Cd;
    Lift = q * Cl;
    
    % Angle of travel (theta)
    theta = atan2(y(4), y(3));
    
    % Equations of Motion (F = ma -> a = F/m)
    ax = (-Drag * cos(theta) - Lift * sin(theta)) / m;
    az = (-Drag * sin(theta) + Lift * cos(theta) - m*g) / m;
    
    dydt = [y(3); y(4); ax; az];
end