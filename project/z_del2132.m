function mpm_simulation()
    % --- Physical Constants & Inputs ---
    g = 9.80665;        % Standard Gravity (m/s^2)
    rho = 1.225;        % Air Density (kg/m^3) - simplified for 900m
    m = 43.35;             % Mass (kg)
    d = 0.155;          % Diameter (m)
    Ix = 0.15;          % Axial Moment of Inertia (kg*m^2)
    omega = 1500;       % Spin rate (rad/s)
    Cma = 3.5;          % Pitching moment coefficient gradient
    Cd = 0.25;          % Drag coefficient
    Cl_alpha = 2.0;     % Lift coefficient gradient
    
    % --- Initial Conditions (10 milliems) ---
    v0 = 700;           % Muzzle velocity (m/s)
    theta0 = 10 * (2*pi/6400); % Elevation in NATO Mils
    
    % State Vector: [x, y, z, vx, vy, vz]
    y0 = [0; 0; 0; v0*cos(theta0); 0; v0*sin(theta0)];
    tspan = [0 10];     % Max time 10s
    dt = 0.01;          % Time step
    
    % --- Run RK4 Solver ---
    [t, states] = rk4_solver(@get_derivatives, tspan, y0, dt);
    
    % --- Stop at Zero Altitude (h=0) ---
    impact_idx = find(states(:,3) < 0, 1);
    final_states = states(1:impact_idx, :);
    
    % Plotting Results
    plot(final_states(:,1), final_states(:,3));
    grid on; xlabel('Range (m)'); ylabel('Altitude (m)');
    title('Modified Point Mass Trajectory');
    fprintf('Final Range: %.2f m\n', final_states(end,1));

    % --- Nested Derivative Function ---
    function dydt = get_derivatives(t, y)
        pos = y(1:3);
        vel = y(4:6);
        v_mag = norm(vel);
        
        % 1. Gravity Vector
        accel_g = [0; 0; -g];
        
        % 2. Yaw of Repose Calculation (Based on your formula)
        % Acceleration vector (approx by gravity for delta_p)
        dv_dt = accel_g; 
        cp = cross(vel, dv_dt); % Cross product (v x dv/dt)
        
        % delta_p formula scale factor
        numerator = 8 * Ix * omega;
        denominator = pi * rho * d^3 * Cma * v_mag^4;
        delta_p_vec = -(numerator / denominator) * cp;
        
        % 3. Aerodynamic Forces
        q = 0.5 * rho * v_mag^2; % Dynamic pressure
        S = pi * (d/2)^2;        % Reference area
        
        % Drag (Opposite to velocity)
        F_drag = - (q * S * Cd) * (vel / v_mag);
        
        % Lift (Perpendicular to velocity, in direction of yaw)
        % For MPM, Lift is usually: q * S * Cl_alpha * delta_p
        % We use the direction of the yaw vector cross velocity
        lift_dir = cross(cross(vel, delta_p_vec), vel);
        lift_dir = lift_dir / norm(lift_dir);
        F_lift = (q * S * Cl_alpha * norm(delta_p_vec)) * lift_dir;
        
        % 4. Total Acceleration
        accel_total = accel_g + (F_drag + F_lift) / m;
        
        dydt = [vel; accel_total];
    end
end

% --- Standard RK4 Implementation ---
function [t, y] = rk4_solver(func, tspan, y0, dt)
    t = tspan(1):dt:tspan(2);
    y = zeros(length(t), length(y0));
    y(1,:) = y0';
    for i = 1:(length(t)-1)
        k1 = func(t(i), y(i,:)');
        k2 = func(t(i)+dt/2, y(i,:)' + dt/2*k1);
        k3 = func(t(i)+dt/2, y(i,:)' + dt/2*k2);
        k4 = func(t(i)+dt, y(i,:)' + dt*k3);
        y(i+1,:) = y(i,:) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)';
        if y(i+1,3) < 0, break; end % Stop condition
    end
    t = t(1:i+1); y = y(1:i+1,:);
end