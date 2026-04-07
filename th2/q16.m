clf; clear all; clc;
%%
q14_15; % get lqr and velo controllers
clf;
quartercar_withtiredamping
quartercar_withtiredamping_cl_lqr
quartercar_withouttiredamping_cl_lqr
%%
roadprofile;
% Prepare the input: Road velocity
dt = t(2) - t(1);
y_dot = diff([hsum(1) hsum]) / dt; % Numerical derivative of road height
y_dot = [zeros(size(y_dot)); y_dot];


%% Simulation
[y_passive, t_out, x_passive] = lsim(quartercar_withtiredamping, y_dot, t);
[y_active, t_out, x_active]   = lsim(quartercar_withtiredamping_cl_lqr, y_dot, t);
clf;
subplot(2,1,1);
plot(t, y_passive(:,1), 'r', t, y_active(:,1), 'b');
title('Ride Comfort: Body Acceleration'); ylabel('m/s^2'); legend('Passive','Active');

subplot(2,1,2);
plot(t, y_passive(:,2), 'r', t, y_active(:,2), 'b');
title('Suspension Working Space: Rattle Space'); ylabel('m');