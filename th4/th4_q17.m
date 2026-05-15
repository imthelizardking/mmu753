%% City Driving Cycle - ECE-15 (Urban part of NEDC)
%  Standard European city driving cycle used for emissions and consumption testing
%  Total duration : 195 seconds (one urban cycle)
%  Max speed      : 50 km/h
%  Distance       : ~1.013 km per cycle
%  Repeated 4x in full NEDC urban phase (UDC = 4 x ECE-15)
%
%  Reference: UN ECE Regulation 83 / EU Directive 70/220/EEC
%
%% --- Vehicle mass placeholder (used in road load plot above) ---
% Set your actual vehicle mass here if not defined elsewhere
m_veh = 1500;   % [kg]
%% --- Single ECE-15 Cycle (195 seconds) ---
% Format: [time(s), speed(km/h)]
% Each row is a waypoint; linear interpolation between points

ECE15_waypoints = [
%  time  speed
    0,    0;
   11,    0;      % idle
   15,   15;      % accelerate
   23,   15;      % cruise at 15 km/h
   28,    0;      % brake
   39,    0;      % idle
   49,   32;      % accelerate
   54,   32;      % cruise at 32 km/h
   56,   35;
   61,   35;      % cruise at 35 km/h
   65,    0;      % brake
   76,    0;      % idle
   87,   50;      % accelerate to 50 km/h
  100,   50;      % cruise at 50 km/h
  111,    0;      % brake
  122,    0;      % idle (end of cycle)
  195,    0;      % hold until 195s (padding for repeat)
];

%% --- Interpolate to 1-second timestep ---
dt        = 1;                                      % timestep [s]
t_single  = 0:dt:194;                              % one cycle: 0 to 194s
V_single  = interp1(ECE15_waypoints(:,1), ...
                    ECE15_waypoints(:,2), ...
                    t_single, 'linear');            % [km/h]

%% --- Full UDC: 4 repetitions of ECE-15 (780 seconds) ---
n_repeat  = 4;
t_UDC     = 0:dt:(195*n_repeat - 1);              % 0 to 779s
V_UDC     = repmat(V_single, 1, n_repeat);         % [km/h]

%% --- Select which cycle to use ---
% Options: 'single' | 'UDC'
cycle_mode = 'UDC';

if strcmp(cycle_mode, 'single')
    t     = t_single;
    V_kmh = V_single;
else
    t     = t_UDC;
    V_kmh = V_UDC;
end

V_ms = V_kmh / 3.6;                               % convert to [m/s]

%% --- Cycle Statistics ---
T         = t(end);
dist_km   = trapz(t, V_ms) / 1000;               % total distance [km]
V_avg     = mean(V_ms(V_ms > 0)) * 3.6;          % average moving speed [km/h]
V_max     = max(V_kmh);                           % maximum speed [km/h]
n_stops   = sum(diff(V_kmh == 0) == -1);          % number of stop events

fprintf('=== City Driving Cycle: ECE-15 (%s) ===\n', cycle_mode);
fprintf('Duration         : %.0f s\n',   T);
fprintf('Total distance   : %.3f km\n',  dist_km);
fprintf('Max speed        : %.0f km/h\n', V_max);
fprintf('Avg moving speed : %.1f km/h\n', V_avg);
fprintf('Number of stops  : %.0f\n',     n_stops);

%% --- Acceleration and Jerk ---
dVdt   = gradient(V_ms, dt);                      % acceleration [m/s^2]
a_max  = max(dVdt);
a_min  = min(dVdt);
fprintf('Max acceleration : %.2f m/s^2\n', a_max);
fprintf('Max deceleration : %.2f m/s^2\n', a_min);

%% --- Plot ---
figure(1);
subplot(3,1,1);
plot(t, V_kmh, 'b-', 'LineWidth', 1.4);
xlabel('Time (s)'); ylabel('Speed (km/h)');
title(sprintf('ECE-15 City Driving Cycle (%s)', cycle_mode));
grid on; ylim([0 60]);

subplot(3,1,2);
plot(t, dVdt, 'r-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
title('Acceleration Profile');
yline(0, 'k--'); grid on;

subplot(3,1,3);
P_road = (m_veh * 9.81 * 0.01 + 0.5 * 1.225 * 2.0 * 0.30 * V_ms.^2) .* V_ms;
plot(t, P_road/1000, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Road Load Power (kW)');
title('Instantaneous Road Load Power');
grid on;

%% --- Ready for use in other scripts ---
% Variables available after running this script:
%   t       : time vector          [s]
%   V_kmh   : speed vector         [km/h]
%   V_ms    : speed vector         [m/s]
%   dVdt    : acceleration vector  [m/s^2]
%   dt      : timestep             [s]
%
% Plug directly into:
%   - stopgo_average_power.m  (replace t and V_kmh)
%   - Your Simulink model     (use as 'From Workspace' block input)
%   - Your ACC algorithm      (use V_kmh as reference speed)

%% --- Export for Simulink (From Workspace block format) ---
% Simulink 'From Workspace' block expects a struct with time and signals
drive_cycle.time    = t';
drive_cycle.signals.values = V_kmh';
drive_cycle.signals.dimensions = 1;
% In Simulink: From Workspace block -> Variable name: drive_cycle

