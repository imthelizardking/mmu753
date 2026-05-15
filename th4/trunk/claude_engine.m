%% Engine Lookup Table Data - 1.6L Naturally Aspirated Passenger Car
%  2D Lookup Table: f(RPM, Throttle Position) -> Torque [Nm] / Efficiency [%]
%  Use with Simulink 2-D Lookup Table block or interpn/interp2 in MATLAB

%% Breakpoint Vectors
RPM_bp = [800, 1200, 1600, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000]; % [RPM]

TPS_bp = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]; % Throttle Position [%]

%% Torque Table [Nm]
%  Rows    -> Throttle Position (TPS_bp)
%  Columns -> Engine Speed      (RPM_bp)

Torque_Nm = [
%  800   1200  1600  2000  2500  3000  3500  4000  4500  5000  5500  6000
     8,   12,   16,   18,   19,   20,   19,   18,   17,   15,   13,   10;  % TPS 10%
    16,   25,   33,   38,   40,   42,   40,   38,   36,   32,   28,   22;  % TPS 20%
    22,   38,   50,   58,   62,   65,   63,   60,   57,   52,   45,   35;  % TPS 30%
    28,   50,   68,   80,   86,   90,   88,   84,   80,   73,   63,   50;  % TPS 40%
    32,   62,   86,  100,  108,  112,  110,  106,  101,   93,   81,   65;  % TPS 50%
    36,   73,  100,  118,  128,  133,  131,  126,  120,  111,   97,   78;  % TPS 60%
    38,   82,  113,  134,  145,  150,  149,  144,  137,  127,  111,   89;  % TPS 70%
    40,   88,  122,  145,  157,  162,  161,  156,  149,  138,  121,   97;  % TPS 80%
    41,   92,  128,  152,  164,  169,  168,  163,  156,  144,  127,  102;  % TPS 90%
    42,   95,  132,  157,  169,  174,  173,  168,  161,  149,  131,  105;  % TPS 100%
];

%% Power Table [kW]  (derived from Torque and RPM)
%  P [kW] = T [Nm] * omega [rad/s] / 1000
%  omega = RPM * 2*pi / 60

omega = RPM_bp * 2 * pi / 60; % angular velocity [rad/s]
Power_kW = (Torque_Nm .* omega) / 1000; % [kW], broadcast over rows

%% Brake Thermal Efficiency Table [%]
%  Rows    -> Throttle Position (TPS_bp)
%  Columns -> Engine Speed      (RPM_bp)
%
%  Peak efficiency ~36% around 2500-3500 RPM at high load (BSFC island)
%  Low efficiency at idle/low load and very high RPM (pumping + friction losses)

Efficiency_pct = [
%  800   1200  1600  2000  2500  3000  3500  4000  4500  5000  5500  6000
   8.0,  9.0, 10.0, 11.0, 11.5, 12.0, 11.5, 11.0, 10.5, 10.0,  9.0,  8.0;  % TPS 10%
  11.0, 14.0, 17.0, 19.0, 20.0, 21.0, 20.5, 19.5, 18.5, 17.0, 15.0, 12.0;  % TPS 20%
  15.0, 19.0, 23.0, 26.0, 28.0, 29.0, 28.5, 27.5, 26.0, 24.0, 21.0, 17.0;  % TPS 30%
  19.0, 24.0, 28.0, 31.0, 33.0, 34.0, 33.5, 32.5, 31.0, 28.5, 25.0, 20.0;  % TPS 40%
  22.0, 27.0, 31.0, 33.5, 35.0, 35.5, 35.0, 34.0, 32.5, 30.0, 26.5, 22.0;  % TPS 50%
  24.0, 29.0, 32.5, 34.5, 35.5, 36.0, 35.5, 34.5, 33.0, 30.5, 27.0, 22.5;  % TPS 60%
  25.0, 30.0, 33.0, 35.0, 36.0, 36.5, 36.0, 35.0, 33.5, 31.0, 27.5, 23.0;  % TPS 70%
  25.5, 30.5, 33.5, 35.5, 36.5, 37.0, 36.5, 35.5, 34.0, 31.5, 28.0, 23.0;  % TPS 80%
  25.5, 31.0, 34.0, 36.0, 36.5, 37.0, 36.5, 35.5, 34.0, 31.5, 28.0, 23.0;  % TPS 90%
  25.0, 31.0, 34.0, 36.0, 36.5, 37.0, 36.5, 35.5, 34.0, 31.5, 28.0, 23.0;  % TPS 100%
];

%% Example: 2D Interpolation (equivalent to Simulink Lookup Table block)
% Query point: RPM = 2750, TPS = 55%
rpm_query = 2750;
tps_query = 55;

torque_interp = interp2(RPM_bp, TPS_bp, Torque_Nm, rpm_query, tps_query, 'linear');
power_interp  = interp2(RPM_bp, TPS_bp, Power_kW,  rpm_query, tps_query, 'linear');
eff_interp    = interp2(RPM_bp, TPS_bp, Efficiency_pct, rpm_query, tps_query, 'linear');

fprintf('At RPM=%.0f, TPS=%.0f%%:\n', rpm_query, tps_query);
fprintf('  Torque     = %.1f Nm\n',  torque_interp);
fprintf('  Power      = %.2f kW\n',  power_interp);
fprintf('  Efficiency = %.1f %%\n',  eff_interp);

%% Optional: Surface Plot
figure;
subplot(1,3,1);
surf(RPM_bp, TPS_bp, Torque_Nm);
xlabel('RPM'); ylabel('Throttle (%)'); zlabel('Torque (Nm)');
title('Engine Torque Map'); colormap jet; colorbar; shading interp;

subplot(1,3,2);
surf(RPM_bp, TPS_bp, Power_kW);
xlabel('RPM'); ylabel('Throttle (%)'); zlabel('Power (kW)');
title('Engine Power Map'); colormap jet; colorbar; shading interp;

subplot(1,3,3);
surf(RPM_bp, TPS_bp, Efficiency_pct);
xlabel('RPM'); ylabel('Throttle (%)'); zlabel('Efficiency (%)');
title('Brake Thermal Efficiency Map'); colormap jet; colorbar; shading interp;