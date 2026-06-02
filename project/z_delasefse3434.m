fprintf('--- Diagnostics ---\n');
fprintf('gamma_A: %.4f, gamma_D: %.4f\n', gamma_out_A, gamma_out_D);
fprintf('||K_A||_inf: %.2f, ||K_D||_inf: %.2f\n', norm(K_A,'inf'), norm(K_D,'inf'));

% Control force RMS on each road
[~,~,x_A_on_A] = lsim(connect(qcar_open_A, tf(1,[1/60 1],'InputName','u','OutputName','fs'), ...
                              K_A, "r", "fs"), r_iso_A, info_A.t);
% (or simpler — use the existing closed-loop and extract fs)

% Or simplest: check Wroad DC gain for both
fprintf('Wroad DC for A: %.4e\n', rms_road_A);  % the rms_road you used when synthesizing K_A
fprintf('Wroad DC for D: %.4e\n', rms_road_D);  % the rms_road you used when synthesizing K_D
fprintf('Ratio D/A: %.2fx\n', rms_road_D/rms_road_A);