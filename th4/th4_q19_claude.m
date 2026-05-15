% clear all; clf;
%% engine
% Gradeability condition: constant speed on maximum slope
v_grade   = 30 / 3.6;          % [m/s]
alpha     = atan(0.30);         % slope angle [rad]

F_grade   = m_eqv*g*sin(alpha) ...  % grade resistance
          + m_eqv*g*Cr*cos(alpha) ...% rolling resistance
          + 0.5*rho*Cd*A*v_grade^2; % aero drag

P_eng_min = F_grade * v_grade / 1000;  % [kW]

F_vmax    = m_eqv*g*Cr + 0.5*rho*Cd*A*(v_max)^2;
P_eng_vmax = F_vmax * v_max / 1000;   % [kW]

P_eng = max(P_eng_min, P_eng_vmax);
%% motor
% Simplified acceleration sizing
v_final = 100 / 3.6;           % [m/s]
t_acc   = 10;                   % [s] target time

% Average force needed (ignoring drag for first estimate)
a_avg     = v_final / t_acc;   % average acceleration [m/s²]
F_acc     = m_eqv * a_avg;         % [N]

P_total   = F_acc * (v_final/2) / 1000;  % [kW] at average speed

% Motor provides the peak power on top of engine
P_motor_min = P_total - P_eng_min;       % [kW]

F_brake_max  = m_eqv * 0.3 * 9.81;          % assume 0.3g max regen decel
P_regen_max  = F_brake_max * v_max / 1000;  % [kW]

P_motor_min  = max(P_motor_min, P_regen_max);

%% battery
% Assume a target EV range (e.g. 30–50 km) or given in homework
d_ev        = 40;               % [km] target electric range
P_avg       = 15;               % [kW] average power demand on drive cycle

E_batt_min  = P_avg * (d_ev / v_avg);   % [kWh]
% Add usable SOC window (e.g. SOC operates between 0.2 and 0.8)
E_batt_total = E_batt_min / (SOC_high - SOC_low);  % [kWh]

% Battery internal resistance model: P = V * I, with voltage sag
% Simple estimate: battery must supply P_motor at max discharge rate
C_rate_max  = 3;                % typical Li-ion max C-rate
E_batt_power = P_motor_min / C_rate_max;  % [kWh]

E_batt = max(E_batt_total, E_batt_power);  % [kWh]


%% acc. sizing
%% Traction Motor Power Rating
%  Formula from lecture:
%  P_t = (gamma*m / 2*t_a) * (Vf^2 + Vb^2) + (2/3)*m*g*f*Vf + (1/5)*rho*S*Cx*Vf^3

% --- Vehicle Parameters (adjust to your car) ---
m_eqv  = 1305.3;       % Vehicle mass                        [kg]
t_a    = 10;         % Time to accelerate 0 -> Vf          [s]
Vf     = 100/3.6;    % Final speed (e.g. 100 km/h)         [m/s]
Vb     = 40/3.6;     % Base speed (constant torque -> constant power transition) [m/s]

% --- Road & Aero Parameters ---
g      = 9.81;       % Gravitational acceleration          [m/s^2]
f      = 0.015;      % Rolling resistance coefficient      [-]
rho    = 1.225;      % Air density                         [kg/m^3]
S      = 1.2;        % Frontal area                        [m^2]
Cx     = 0.5;       % Drag coefficient                    [-]

% --- Motor Power Rating Calculation ---
P_inertia  = (m_eqv / (2 * t_a)) * (Vf^2 + Vb^2);   % Inertia term      [W]
P_rolling  = (2/3) * m_eqv * g * f * Vf;                      % Rolling res. term [W]
P_aero     = (1/5) * rho * S * Cx * Vf^3;                 % Aerodynamic term  [W]

P_t        = P_inertia + P_rolling + P_aero;               % Total power       [W]
P_t_kW     = P_t / 1000;                                   % Convert to        [kW]

% --- Display Results ---
fprintf('=== Traction Motor Power Rating ===\n');
fprintf('Inertia term   : %6.2f kW\n', P_inertia/1000);
fprintf('Rolling term   : %6.2f kW\n', P_rolling/1000);
fprintf('Aero term      : %6.2f kW\n', P_aero/1000);
fprintf('-----------------------------------\n');
fprintf('Total P_t      : %6.2f kW\n', P_t_kW);