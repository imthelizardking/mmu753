clf; clc;
%% name changes
v_ms = out.logsout{1}.Values.Data; v_ms = v_ms / 3.6;
t = out.tout;
P_eng = out.logsout{23}.Values.Data;
eta_engine = out.logsout{28}.Values.Data;
SOC = out.logsout{17}.Values.Data;
SOC_high = SOC_max;
SOC_low = SOC_min;
engine_on_off = out.logsout{18}.Values.Data;
RPM = out.logsout{25}.Values.Data;



T_eng = out.logsout{27}.Values.Data;
P_batt = out.logsout{16}.Values.Data;
%% Motor speed from vehicle speed
r_wheel    = 0.5;       % [m]  — your wheel radius
G          = 5;         % [-]  — your gear ratio

% Angular velocity of motor [rad/s]
omega_motor = (v_ms / r_wheel) * G;        % [rad/s]

% Motor speed in RPM
RPM_motor   = omega_motor * 60 / (2*pi);   % [RPM]
% From your simulation outputs: P_eng [kW], Efficiency_torque map, RPM, Torque
RPM_bp_em = RPM_motor;
% Fuel power (power extracted from fuel)
P_fuel = P_eng / (eta_engine / 100);    % [kW]

%% Motor torque from P_batt and vehicle speed

r_wheel     = 0.5;          % [m]
G           = 5;            % gear ratio

% Motor angular velocity [rad/s]
omega_motor = (v_ms / r_wheel) * G;

% Clamp to avoid division by zero at standstill
omega_safe  = max(omega_motor, 0.1);


%%
% Make sure all vectors are same orientation
P_eng        = P_eng(:);           % force column vector
engine_on_off = engine_on_off(:);  % force column vector
RPM      = RPM(:);
T_eng        = T_eng(:);

% Recompute eta_engine
eta_engine = interp2(RPM_bp, Torque_bp, Efficiency_torque, ...
                     RPM, T_eng, 'linear', 0) / 100;

% Initialize P_fuel as zeros column vector
P_fuel = zeros(size(P_eng));       % 3664x1

% Fill only engine-on points
idx_on = engine_on_off == 1 & eta_engine > 0.01;
P_fuel(idx_on) = (P_eng(idx_on) .* 1000) ./ eta_engine(idx_on);  % [W]

% Motor torque [Nm]
T_motor     = (P_batt * 1000) ./ omega_safe;   % P_batt in kW -> *1000 for W
% Fuel energy consumed over simulation
E_fuel_kJ  = trapz(t, P_fuel) ;        % [kJ]  (P in kW, t in s -> kJ)
E_fuel_kWh = E_fuel_kJ / 3600;         % [kWh]

% Convert to liters (gasoline: 1L = 8.9 kWh ~ 32 MJ lower heating value)
LHV_gasoline = 32000;                   % [kJ/L]
fuel_liters  = E_fuel_kJ / LHV_gasoline;  % [L]

% Fuel consumption per 100km
dist_km      = trapz(t, v_ms) / 1000;  % total distance [km]
fuel_per_100km = fuel_liters / dist_km * 100;  % [L/100km]

fprintf('Fuel consumption : %.2f L\n', fuel_liters);
fprintf('Fuel per 100km   : %.2f L/100km\n', fuel_per_100km);
figure;
plot(t, SOC * 100, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('SOC (%)');
title('Battery SOC Profile');
grid on; ylim([0 100]);
yline(SOC_high*100, 'r--', 'SOC high');
yline(SOC_low*100,  'g--', 'SOC low');
% From simulation: RPM vector, T_eng vector (engine torque over time)
% Only plot points where engine was ON
idx_on = engine_on_off == 1;

figure;
% Background: efficiency map contour
contourf(RPM_bp, Torque_bp, Efficiency_torque, 20);
colormap turbo; colorbar;
xlabel('Engine Speed (RPM)'); ylabel('Torque (Nm)');
title('ICE Operating Points');
hold on;

% Overlay actual operating points
scatter(RPM(idx_on), T_eng(idx_on), 10, 'w', 'filled', ...
        'MarkerFaceAlpha', 0.5);
legend('Efficiency map', 'Operating points');
% From simulation: RPM_motor vector, T_motor vector
% T_motor is positive (motoring) or negative (regen)

figure;
scatter(RPM_motor(T_motor > 0), T_motor(T_motor > 0), 10, 'b', 'filled');
hold on;
scatter(RPM_motor(T_motor < 0), T_motor(T_motor < 0), 10, 'r', 'filled');
xlabel('Motor Speed (RPM)'); ylabel('Motor Torque (Nm)');
title('EM Operating Points');
yline(0, 'k--');
legend('Motoring', 'Regenerating');
grid on;

% Motoring points (T > 0)
idx_motor = T_motor > 0;
scatter(RPM_motor(idx_motor), T_motor(idx_motor), 10, 'w', 'filled');

% Regen points (T < 0)
idx_regen = T_motor < 0;
scatter(RPM_motor(idx_regen), T_motor(idx_regen), 10, 'y', 'filled');

%% Debug prints
fprintf('--- Debug ---\n');
fprintf('P_eng max        : %.6f (units?)\n',  max(P_eng));
fprintf('P_eng when on    : %.6f\n',  max(P_eng(engine_on_off==1)));
fprintf('RPM_eng range    : %.1f to %.1f\n', min(RPM(engine_on_off==1)), max(RPM(engine_on_off==1)));
fprintf('T_eng range      : %.2f to %.2f Nm\n', min(T_eng(engine_on_off==1)), max(T_eng(engine_on_off==1)));
fprintf('eta_engine range : %.4f to %.4f\n', min(eta_engine(engine_on_off==1)), max(eta_engine(engine_on_off==1)));
fprintf('P_fuel max       : %.4f W\n',  max(P_fuel));
fprintf('E_fuel_kJ        : %.6f kJ\n', E_fuel_kJ);
fprintf('fuel_liters      : %.6f L\n',  fuel_liters);
fprintf('dist_km          : %.4f km\n', dist_km);
fprintf('v_ms max         : %.4f m/s\n', max(v_ms));
fprintf('t range          : %.1f to %.1f s\n', t(1), t(end));



yline(0, 'w--', 'LineWidth', 1.5);
legend('Efficiency map', 'Motoring', 'Regen');