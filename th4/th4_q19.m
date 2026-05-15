clear all; clf;
%% acc. sizing
% Traction Motor Power Rating
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

%% Engine/Motor Power Rating for Constant Speed Cruising
%  Formula from lecture:
%  P_e/g = V / (eta_t * eta_m) * (m*g*f + 0.5*rho*S*Cx*V^2)

% --- Efficiency Parameters ---
eta_t  = 0.8;       % Transmission efficiency             [-]
eta_m  = 0.9;       % Motor/engine efficiency             [-]

% --- Constants ---
% --- Single Speed Calculation ---
V_kmh  = 130;                   % Cruise speed             [km/h]
V      = V_kmh / 3.6;           % Convert to               [m/s]

F_rolling = m_eqv * g * f;                      % Rolling resistance force  [N]
F_aero    = 0.5 * rho * S * Cx * V^2;      % Aerodynamic drag force    [N]
F_total   = F_rolling + F_aero;             % Total road load force     [N]

P_eg      = V / (eta_t * eta_m) * F_total; % Required power            [W]
P_eg_kW   = P_eg / 1000;                   % Convert to                [kW]

fprintf('=== Cruise Power Rating at %.0f km/h ===\n', V_kmh);
fprintf('Rolling resistance : %6.1f N\n',  F_rolling);
fprintf('Aerodynamic drag   : %6.1f N\n',  F_aero);
fprintf('Total road load    : %6.1f N\n',  F_total);
fprintf('----------------------------------------\n');
fprintf('Required power P   : %6.2f kW\n', P_eg_kW);

% % --- Power vs Speed Curve (replicates lecture plot) ---
% V_range_kmh = 0:1:160;                     % Speed range   [km/h]
% V_range     = V_range_kmh / 3.6;           % Convert to    [m/s]
% 
% F_roll_vec  = m_eqv * g * f * ones(size(V_range));          % constant
% F_aero_vec  = 0.5 * rho * S * Cx * V_range.^2;         % speed dependent
% F_total_vec = F_roll_vec + F_aero_vec;
% 
% P_eg_vec    = V_range ./ (eta_t * eta_m) .* F_total_vec;
% P_eg_kW_vec = P_eg_vec / 1000;
% 
% figure;
% plot(V_range_kmh, P_eg_kW_vec, 'b-', 'LineWidth', 2);
% xlabel('Vehicle speed (km/h)');
% ylabel('Load power (kW)');
% title('Engine/Motor Cruise Power vs Speed');
% grid on;
% legend(sprintf('M=%.0fkg, f=%.2f, Cd=%.1f, Af=%.1fm²', m_eqv, f, Cx, S));
% xlim([0 160]); ylim([0 45]);
% 
% % Mark the example point from lecture
% hold on;
% plot(130, P_eg_kW, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% text(133, P_eg_kW+1, sprintf('%.1f kW at %d km/h', P_eg_kW, V_kmh), ...
%     'FontSize', 10, 'Color', 'r');

%% Average Load Power - Stop and Go Driving
%  Formula from lecture:
%  P_ave = (1/T) * integral[(m*g*f + 0.5*rho*S*Cx*V^2)*V dt] 
%        + (1/T) * integral[gamma*m*(dV/dt) dt]
%
%  The second term (kinetic energy term) is zero if full regen is available,
%  and greater than zero if regen is partial or absent.
%% --- Drive Cycle Input ---
% Replace this with your actual drive cycle (e.g. WLTP, NEDC, FTP-75)
%  Format: time vector [s] and speed vector [m/s]
%
%  Example: simple trapezoidal stop-and-go cycle
%  0->10s: accelerate to 50 km/h | 10->30s: cruise | 30->40s: brake to 0

dt        = 1;                          % timestep                  [s]
t         = 0:dt:120;                   % time vector               [s]
V_kmh     = zeros(size(t));

% Build a simple repeated stop-and-go pattern
cycle     = [linspace(0,50,10), 50*ones(1,20), linspace(50,0,10), zeros(1,10)];
n_repeat  = floor(length(t) / length(cycle));
remainder = length(t) - n_repeat*length(cycle);
V_kmh     = [repmat(cycle, 1, n_repeat), cycle(1:remainder+1)];
V_kmh     = V_kmh(1:length(t));

V         = V_kmh / 3.6;               % convert to                [m/s]
T         = t(end) - t(1);             % total cycle duration      [s]

% --- Term 1: Road Load Power Integral ---
%  (1/T) * integral[(m*g*f + 0.5*rho*S*Cx*V^2) * V dt]
F_rolling  = m_eqv * g * f;                             % constant term [N]
F_aero     = 0.5 * rho * S * Cx * V.^2;            % speed dependent [N]
P_roadload = (F_rolling + F_aero) .* V;             % instantaneous road load power [W]

Term1      = (1/T) * trapz(t, P_roadload);          % [W]

% --- Term 2: Kinetic Energy / Inertia Integral ---
%  (1/T) * integral[gamma*m*(dV/dt) dt]
%  Note: if full regen, this term = 0 (energy recovered during braking)
%        if no regen,   this term > 0 (braking energy lost as heat)

dVdt       = gradient(V, dt);                       % numerical dV/dt [m/s^2]

% Full regen: only count positive acceleration (dV/dt > 0)
% No regen:   count all acceleration, braking energy is lost
full_regen = true;                                  % set false if no regen

if full_regen
    dVdt_effective = max(dVdt, 0);                  % only acceleration phases
else
    dVdt_effective = dVdt;                          % all phases
end

P_inertia  = m_eqv * dVdt_effective .* V;      % instantaneous inertia power [W]
Term2      = (1/T) * trapz(t, P_inertia);          % [W]

% --- Average Power ---
P_ave      = Term1 + Term2;                         % [W]
P_ave_kW   = P_ave / 1000;                         % [kW]

% --- Display Results ---
fprintf('=== Average Load Power - Stop and Go ===\n');
fprintf('Cycle duration        : %.0f s\n',   T);
fprintf('Average speed         : %.1f km/h\n', mean(V)*3.6);
fprintf('Road load term (T1)   : %.2f kW\n',  Term1/1000);
fprintf('Inertia term   (T2)   : %.2f kW\n',  Term2/1000);
fprintf('Full regen assumed    : %s\n',        mat2str(full_regen));
fprintf('----------------------------------------\n');
fprintf('Average power P_ave   : %.2f kW\n',  P_ave_kW);

%%--- Engine/Generator sizing from P_ave ---
%  Engine must produce >= P_ave to maintain energy balance
eta_t      = 0.92;
eta_m      = 0.91;
P_eng_min  = P_ave_kW / (eta_t * eta_m);          % [kW]
fprintf('Min engine power      : %.2f kW\n',  P_eng_min);

% % --- Plot ---
% figure;
% subplot(2,1,1);
% plot(t, V_kmh, 'b-', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Speed (km/h)');
% title('Drive Cycle'); grid on;
% 
% subplot(2,1,2);
% plot(t, P_roadload/1000, 'b-', 'LineWidth', 1.5); hold on;
% plot(t, P_inertia/1000,  'r--', 'LineWidth', 1.5);
% plot(t, (P_roadload + P_inertia)/1000, 'k-', 'LineWidth', 1.5);
% yline(P_ave_kW, 'g--', sprintf('P_{ave} = %.1f kW', P_ave_kW), 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Power (kW)');
% title('Instantaneous vs Average Power');
% legend('Road load', 'Inertia', 'Total', 'P_{ave}');
% grid on;