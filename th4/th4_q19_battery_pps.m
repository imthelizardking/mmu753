%% Battery Sizing for Hybrid Vehicle
%  Two constraints: Energy constraint and Power constraint
%  Final size is the larger of the two
%
%  Reference: Lecture methodology for HEV component sizing

%% --- Vehicle & Motor Parameters (from your previous sizing) ---
m           = 1305.3;      % vehicle mass                    [kg]
P_motor_kW  = 69.80;       % rated motor power (from sizing) [kW]
P_eng_kW    = 60.0;        % engine power                    [kW]

%% --- Battery Operating Parameters ---
SOC_high    = 0.8;         % upper SOC limit                 [-]
SOC_low     = 0.4;         % lower SOC limit                 [-]
SOC_usable  = SOC_high - SOC_low;  % usable SOC window       [-]  = 0.4

eta_batt    = 0.92;        % round-trip battery efficiency   [-]
V_nominal   = 300;         % nominal battery pack voltage     [V]  (typical HEV)

%% ========================================================
%% CONSTRAINT 1 — Energy Constraint (EV range)
%% ========================================================
%  Battery must store enough energy for a minimum EV-only range
%  P_avg from your stop-and-go calculation

P_avg_kW    = 3.5;         % average power demand on city cycle [kW]
                           % (from your stopgo_average_power.m output)
d_ev_km     = 20;          % target EV-only range            [km]
v_avg_ms    = 20/3.6;      % average city speed              [m/s]

% Time to cover EV range
t_ev        = (d_ev_km * 1000) / v_avg_ms;   % [s]

% Energy needed at wheel
E_wheel_kWh = P_avg_kW * t_ev / 3600;        % [kWh]

% Gross battery energy needed (account for usable window and efficiency)
E_batt_energy = E_wheel_kWh / (SOC_usable * eta_batt);  % [kWh]

fprintf('=== Constraint 1: Energy ===\n');
fprintf('Target EV range       : %.0f km\n',    d_ev_km);
fprintf('Average power demand  : %.1f kW\n',    P_avg_kW);
fprintf('Energy at wheel       : %.2f kWh\n',   E_wheel_kWh);
fprintf('Battery size (energy) : %.2f kWh\n\n', E_batt_energy);

%% ========================================================
%% CONSTRAINT 2 — Power Constraint (peak motor power)
%% ========================================================
%  Battery must deliver peak motor power during acceleration
%  and absorb peak regen power during braking
%
%  C-rate: how fast battery can charge/discharge relative to capacity
%  Typical Li-ion HEV battery: 10-30C continuous, 40C peak

C_rate_discharge = 20;     % max continuous discharge C-rate  [-]
C_rate_charge    = 15;     % max continuous charge C-rate     [-]  (regen)

P_regen_max_kW   = 30.0;   % max regen power (from EMS)       [kW]

% From discharge (motor peak power)
E_batt_discharge = P_motor_kW / C_rate_discharge;   % [kWh]

% From charge (regen peak power)
E_batt_charge    = P_regen_max_kW / C_rate_charge;  % [kWh]

E_batt_power     = max(E_batt_discharge, E_batt_charge);  % [kWh]

fprintf('=== Constraint 2: Power ===\n');
fprintf('Motor peak power      : %.1f kW\n',    P_motor_kW);
fprintf('Max C-rate discharge  : %.0f C\n',     C_rate_discharge);
fprintf('Battery size (motor)  : %.2f kWh\n',   E_batt_discharge);
fprintf('Regen peak power      : %.1f kW\n',    P_regen_max_kW);
fprintf('Max C-rate charge     : %.0f C\n',     C_rate_charge);
fprintf('Battery size (regen)  : %.2f kWh\n',   E_batt_charge);
fprintf('Battery size (power)  : %.2f kWh\n\n', E_batt_power);

%% ========================================================
%% FINAL BATTERY SIZE
%% ========================================================
E_batt_final = max(E_batt_energy, E_batt_power);   % [kWh]

% Determine which constraint drove the sizing
if E_batt_energy >= E_batt_power
    driving_constraint = 'Energy (EV range)';
else
    driving_constraint = 'Power (motor/regen)';
end

fprintf('=== Final Battery Size ===\n');
fprintf('Energy constraint     : %.2f kWh\n', E_batt_energy);
fprintf('Power constraint      : %.2f kWh\n', E_batt_power);
fprintf('Driving constraint    : %s\n',        driving_constraint);
fprintf('Final battery size    : %.2f kWh\n', E_batt_final);

%% ========================================================
%% DERIVED BATTERY PARAMETERS
%% ========================================================
% Capacity in Ah (for SOC model)
Q_batt_Ah   = (E_batt_final * 1000) / V_nominal;  % [Ah]
Q_batt_As   = Q_batt_Ah * 3600;                    % [As = Coulombs]

% For your SOC block:
% SOC(i+1) = SOC(i) - P_batt(i)*1000/3600 / (V_nominal * Q_batt_Ah)
% or equivalently:
% SOC(i+1) = SOC(i) - P_batt(i) / (E_batt_final * 3600) * dt

% Peak current
I_peak_A    = (P_motor_kW * 1000) / V_nominal;    % [A]

fprintf('\n=== Derived Battery Parameters ===\n');
fprintf('Nominal voltage       : %.0f V\n',    V_nominal);
fprintf('Capacity              : %.1f Ah\n',   Q_batt_Ah);
fprintf('Peak discharge current: %.0f A\n',    I_peak_A);
fprintf('Usable energy         : %.2f kWh\n',  E_batt_final * SOC_usable);
fprintf('SOC window            : %.0f%% - %.0f%%\n', SOC_low*100, SOC_high*100);

%% ========================================================
%% SUMMARY TABLE
%% ========================================================
fprintf('\n========================================\n');
fprintf('     HEV COMPONENT SIZING SUMMARY\n');
fprintf('========================================\n');
fprintf(' Engine power    : %.1f kW\n',  P_eng_kW);
fprintf(' Motor power     : %.1f kW\n',  P_motor_kW);
fprintf(' Battery energy  : %.2f kWh\n', E_batt_final);
fprintf(' Battery voltage : %.0f V\n',   V_nominal);
fprintf(' Battery capacity: %.1f Ah\n',  Q_batt_Ah);
fprintf('========================================\n');

%% ========================================================
%% USE IN SOC BLOCK
%% ========================================================
% Replace 'Battery_capacity' in your existing SOC code with E_batt_final:
%
% if P_batt(i) > 0   % discharging
%     SOC(i+1) = SOC(i) - P_batt(i) * dt/3600 / (E_batt_final * eta_batt);
% else               % charging
%     SOC(i+1) = SOC(i) - P_batt(i) * dt/3600 * eta_batt / E_batt_final;
% end