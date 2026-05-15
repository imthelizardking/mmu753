%% Engine Efficiency Map - 1.6L Naturally Aspirated Passenger Car
%  2D Lookup Table: Efficiency = f(RPM, Torque)
%  This is equivalent to a BSFC map, re-expressed as efficiency [%]
%  Use directly with Simulink 2-D Lookup Table block
%
%  Inputs  : Engine Speed [RPM], Torque Demand [Nm]
%  Output  : Brake Thermal Efficiency [%]

%% Breakpoint Vectors
RPM_bp    = [800, 1200, 1600, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000]; % [RPM]
Torque_bp = [0, 20, 40, 60, 80, 100, 120, 140, 160, 174];                             % [Nm]

%% Brake Thermal Efficiency Table [%]
%  Rows    -> Torque demand (Torque_bp)
%  Columns -> Engine speed  (RPM_bp)
%
%  Characteristics:
%   - Peak efficiency ~37% around 2500-3500 RPM, 120-160 Nm (high load island)
%   - Very low efficiency at idle / low torque (pumping + friction dominate)
%   - Drops off at high RPM due to friction and breathing losses
%   - Light load rows (0-40 Nm) are typical city driving conditions

Efficiency_torque = [
%   800   1200  1600  2000  2500  3000  3500  4000  4500  5000  5500  6000
    3.0,  4.0,  5.0,  5.5,  6.0,  6.0,  5.5,  5.0,  4.5,  4.0,  3.5,  3.0;  % T =   0 Nm (idle/friction)
    8.0, 11.0, 14.0, 16.0, 17.0, 17.5, 17.0, 16.0, 15.0, 13.5, 11.5,  9.0;  % T =  20 Nm
   13.0, 18.0, 22.0, 25.0, 27.0, 28.0, 27.5, 26.5, 25.0, 22.5, 19.5, 15.0;  % T =  40 Nm
   17.0, 22.0, 27.0, 30.0, 32.0, 33.0, 32.5, 31.5, 30.0, 27.5, 24.0, 19.0;  % T =  60 Nm
   20.0, 26.0, 30.5, 33.5, 35.0, 35.5, 35.0, 34.0, 32.5, 30.0, 26.5, 21.5;  % T =  80 Nm
   22.5, 28.5, 32.5, 35.0, 36.5, 37.0, 36.5, 35.5, 34.0, 31.5, 28.0, 23.0;  % T = 100 Nm
   24.0, 30.0, 33.5, 35.5, 37.0, 37.5, 37.0, 36.0, 34.5, 32.0, 28.5, 23.5;  % T = 120 Nm
   24.5, 30.5, 34.0, 36.0, 37.5, 38.0, 37.5, 36.5, 35.0, 32.5, 29.0, 24.0;  % T = 140 Nm
   24.0, 30.0, 33.5, 36.0, 37.5, 38.0, 37.5, 36.5, 35.0, 32.5, 29.0, 24.0;  % T = 160 Nm
   23.0, 29.5, 33.0, 35.5, 37.0, 37.5, 37.0, 36.0, 34.5, 32.0, 28.5, 23.5;  % T = 174 Nm (peak torque)
];

%% Example: 2D Interpolation
% Query: RPM = 3000, Torque demand = 110 Nm
rpm_query    = 3000;
torque_query = 110;

eff = interp2(RPM_bp, Torque_bp, Efficiency_torque, rpm_query, torque_query, 'linear');
fprintf('At RPM = %.0f, Torque = %.0f Nm -> Efficiency = %.1f%%\n', rpm_query, torque_query, eff);

%% Simulink usage note:
%  - 2-D Lookup Table block
%  - Row breakpoints    : Torque_bp  (connect your torque demand signal)
%  - Column breakpoints : RPM_bp     (connect your engine speed signal)
%  - Table data         : Efficiency_torque
%  - Interpolation      : Linear
%  - Extrapolation      : Clamp (to avoid unrealistic values outside the map)

%% Optional: Surface Plot
figure;
surf(RPM_bp, Torque_bp, Efficiency_torque);
xlabel('Engine Speed (RPM)');
ylabel('Torque (Nm)');
zlabel('Brake Thermal Efficiency (%)');
title('Engine Efficiency Map - 1.6L NA');
colormap turbo; colorbar; shading interp;
hold on;
[~, h] = contour3(RPM_bp, Torque_bp, Efficiency_torque, 10, 'k');
h.LineWidth = 0.8;