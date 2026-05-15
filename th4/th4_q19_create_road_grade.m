%% Road Grade Angle Profile — Simulink From Workspace
%  Output: alpha [rad] — road angle
%  Positive = uphill, Negative = downhill

dt  = 0.01;
t   = (0:dt:1000)';
N   = length(t);

grade = zeros(N, 1);

% [t_start, t_end, grade_percent/100]
segments = [
      0,     80,    0.00;
     80,    160,    0.04;
    160,    200,    0.00;
    200,    280,   -0.04;
    280,    320,    0.00;
    320,    400,    0.07;
    400,    440,    0.00;
    440,    520,   -0.06;
    520,    560,    0.00;
    560,    620,    0.03;
    620,    680,   -0.03;
    680,    720,    0.00;
    720,    800,    0.08;
    800,    860,   -0.07;
    860,    900,    0.00;
    900,    950,    0.05;
    950,   1000,   -0.05;
];

% Apply segments with 2s ramps
t_ramp = 2.0;
for i = 1:size(segments, 1)
    t_s = segments(i,1);
    t_e = segments(i,2);
    g   = segments(i,3);

    idx_up  = t >= t_s & t < t_s + t_ramp;
    idx_mid = t >= t_s + t_ramp & t <= t_e - t_ramp;
    idx_dn  = t > t_e - t_ramp & t <= t_e;

    grade(idx_up)  = grade(idx_up)  + g * (t(idx_up)  - t_s) / t_ramp;
    grade(idx_mid) = g;
    grade(idx_dn)  = grade(idx_dn)  + g * (t_e - t(idx_dn)) / t_ramp;
end

%% Convert to angle
alpha = atan(grade);               % [rad]

%% Simulink From Workspace struct
alpha_signal.time              = t;
alpha_signal.signals.values    = alpha;
alpha_signal.signals.dimensions = 1;

%% Plot
figure;
plot(t, rad2deg(alpha), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Grade angle (deg)');
title('Road Grade Angle Profile');
yline(0,'k--'); grid on;