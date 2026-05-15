clear all; clf;
q7;
%%
% We use the same rhos and R, but we zero out the position gain (K1)
% Define frequency range for plotting
figure(3);
semilogx(f,sys_passive_,'b');
hold on;
figure(4);
semilogx(f,sys_passive_rattlespace,'b');
labels_(1) = "passive";
hold on;
rho1 = [0]; % body acc.
rho2 = [10000]; % rattlespace
temp = [rho1, rho2];
rhos = permute(temp, [2, 3, 1]);
for i = 1:size(rhos, 3)
    % 1. Compute the optimal LQR gains as a baseline
    Q_curr = diag([rhos(1,1,i), rhos(2,1,i)]);
    [K_full, ~, ~] = lqr(A, B, Q_curr, R);

    % 2. QUESTION 8 STEP: Create Simple Velocity Feedback
    % We take ONLY the second gain (the one multiplying x2/velocity)
    k_vel_only = K_full(2);
    K_simple = [0, k_vel_only]; % Set position feedback to zero

    % 3. Compute Closed-Loop Response with Velocity Feedback Only
    % New A matrix: A_cl = A - B * K_simple
    [sys_vel_fb, ~] = bode(A - B*K_simple, L, C, D, 1, w);

    % 4. Process Magnitude Data (Acceleration and Rattle Space)
    for k = 1:100
        % Calculate Acceleration: H(s) * s
        acc_mag = abs(sys_vel_fb(k, 2, 1) * s(k));
        sys_active_velo_acc(k, i) = 20*log10(acc_mag);

        % Calculate Rattle Space: H(s) * s
        rs_mag = abs(sys_vel_fb(k, 1, 1) * s(k));
        sys_active_velo_rattlespace(k, i) = 20*log10(rs_mag);
    end

    % 5. Plot results
    figure(1);
    %figure(3);
    semilogx(f, sys_active_velo_acc(:, i));
    hold on;

    figure(2);
    %figure(4);
    semilogx(f, sys_active_velo_rattlespace(:, i));
    hold on;

    % Update Labels
    %labels_(i+1) = strcat('gain= ', num2str(k_vel_only));
    labels_(i) = strcat('gain= ', num2str(k_vel_only));
end

% % single plot
% figure(3);
% legend(labels_, 'Location', 'best'); % Add all labels at once
% xlabel('Frequency [Hz]');
% title('Body acceleration');
% set(gca, 'XScale', 'log');
% grid minor;
% figure(4);
% legend(labels_, 'Location', 'best'); % Add all labels at once
% xlabel('Frequency [Hz]');
% title('Rattle space');
% set(gca, 'XScale', 'log');
% grid minor;

% plot above LQR
figure(1);
legend([labels, labels_], 'Location', 'best'); % Add all labels at once
%xlabel('Frequency [Hz]');
%title('Body acceleration');
%set(gca, 'XScale', 'log');
%grid minor;
figure(2);
legend([labels, labels_], 'Location', 'best'); % Add all labels at once
%xlabel('Frequency [Hz]');
%title('Rattle space');
%set(gca, 'XScale', 'log');
%grid minor;