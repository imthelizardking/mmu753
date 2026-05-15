clf; clear all; clc;
%% get models
q14_15; % get lqr and velo controllers
clf;
quartercar_withtiredamping
quartercar_withtiredamping_cl_lqr
quartercar_withouttiredamping_cl_lqr
quartercar_withtiredamping_cl_velocityfeedback
quartercar_withouttiredamping_cl_velocityfeedback
%% create road input
roadprofile; % get road input
% Prepare the input: Road velocity
dt = t(2) - t(1);
y = hsum;
y = [zeros(size(y)); y];
y_dot = diff([hsum(1) hsum]) / dt; % Numerical derivative of road height
y_dot = [zeros(size(y_dot)); y_dot];
%% Simulation
[y_passive, t_out, x_passive] = lsim(quartercar_withtiredamping, y, t);
[y_active_withtiredamping_lqr, t_out, x_withtiredamping_lqr]   = lsim(quartercar_withtiredamping_cl_lqr, y, t);
[y_active_withouttiredamping_lqr, t_out, x_withouttiredamping_lqr]   = lsim(quartercar_withouttiredamping_cl_lqr, y, t);
[y_active_withtiredamping_cl_velocityfeedback, t_out, x_withtiredamping_velocityfeedback]   = lsim(quartercar_withtiredamping_cl_velocityfeedback, y, t);
[y_active_withouttiredamping_cl_velocityfeedback, t_out, x_withouttiredamping_velocityfeedback]   = lsim(quartercar_withouttiredamping_cl_velocityfeedback, y, t);
[y_temp t_out, x_Temp] = lsim(temp_sys, hsum, t);


clf;
subplot(2,1,1);
plot(t, y_passive(:,1), 'r',...
     t, y_temp(:,1), 'b');
% plot(t, y_passive(:,1), 'r',...
%      t, y_active_withtiredamping_lqr(:,1), 'b',...
%      t, y_active_withouttiredamping_lqr(:,1), 'g',...
%      t, y_active_withtiredamping_cl_velocityfeedback(:,1), 'm',...
%      t, y_active_withouttiredamping_cl_velocityfeedback(:,1), 'k');
title('Ride Comfort: Body Acceleration'); ylabel('m/s^2');
legend('Passive',...
      'lqr with tire damping', 'lqr without tire damping',...
      'velo fb with tire damping', 'velo fb without tire damping');

subplot(2,1,2);
plot(t, y_passive(:,2), 'r',...
     t, y_temp(:,2), 'b');
% plot(t, y_passive(:,2), 'r',...
%      t, y_active_withtiredamping_lqr(:,2), 'b',...
%      t, y_active_withouttiredamping_lqr(:,2), 'g',...
%      t, y_active_withtiredamping_cl_velocityfeedback(:,2), 'm',...
%      t, y_active_withouttiredamping_cl_velocityfeedback(:,2), 'k');
title('Suspension Working Space: Rattle Space'); ylabel('m');
legend('Passive',...
      'lqr with tire damping', 'lqr without tire damping',...
      'velo fb with tire damping', 'velo fb without tire damping');