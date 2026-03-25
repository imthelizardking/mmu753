clf; 
clear all; clc;
%% create model
% model_quatercar_bd;
model_rajavani_actsus_modifiedhinf;
%% design controllers:
% lqr:
% ro1 = 0.4; ro2 = 0.16; ro3 = 0.4; ro4 = 0.16;
% ro1 = 1.0; ro2 = 1.0; ro3 = 1.0; ro4 = 1.0;
ro1 = 400; ro2 = 16; ro3 = 400; ro4 = 16;
K_lqr = controller_actsus_rajamani(A, B, ro1, ro2, ro3, ro4, ks, ms, bs);
quartercar_cl_lqr = ss(A-B*K_lqr,B_x,C,D);
quartercar_cl_lqr.InputName = {'fs', 'r'};
quartercar_cl_lqr.OutputName = {'body acceleration'; 'rattle space'; 'tire deflection'};

% hinf:
Act = tf(1000,[1/60 1]); Act.InputName = 'u'; Act.OutputName = 'fs';
%% two different wroad approaches:
Wroad = ss(0.07);

V = 50; % Velocity in m/s
Phi = 5e-6; % Road roughness (e.g., "Good" road)
Wroad = tf(sqrt(2*Phi*V), [1, 0.1]); % Velocity-dependent filter

% center_freq = 8; 
% Q = 1; % Quality factor (width of the peak)
% Wroad = tf([center_freq/Q, 0], [1, center_freq/Q, center_freq^2]) * 0.07;

Wroad.u = 'd1'; Wroad.y = 'r';
%%
Wact = 0.8*tf([1 50],[1 500]);  Wact.u = 'u';  Wact.y = 'e1';
Wd2 = ss(0.01);  Wd2.u = 'd2';   Wd2.y = 'Wd2';
Wd3 = ss(0.5);   Wd3.u = 'd3';   Wd3.y = 'Wd3';
HandlingTarget = 0.04 * tf([1/8 1],[1/80 1]);
ComfortTarget = 0.4 * tf([1/0.45 1],[1/150 1]);
Targets = [HandlingTarget ; ComfortTarget];
beta = [0.01];
Wsd = beta / HandlingTarget;
Wsd.u = 'sd';  Wsd.y = 'e3';
Wab = (1-beta) / ComfortTarget;
Wab.u = 'ab';  Wab.y = 'e2';
sdmeas  = sumblk('y1 = sd+Wd2');
abmeas = sumblk('y2 = ab+Wd3');
ICinputs = {'d1';'d2';'d3';'u'};
ICoutputs = {'e1';'e2';'e3';'y1';'y2'};
qcaric = connect(quartercar_x(1:2,:),Act,Wroad,Wact,Wab,Wsd,Wd2,Wd3,...
                 sdmeas,abmeas,ICinputs,ICoutputs);
[K_hinf, ~, gamma] = hinfsyn(qcaric, 2, 1);
K_hinf.InputName = {'sd', 'ab'};
K_hinf.OutputName = {'u'};
quartercar_cl_hinf = connect(quartercar_x, Act, K_hinf, 'r', {'ab'; 'sd'; 'tire deflection'});
%% plot
clf
% bodemag(quartercar('body acceleration','r'),'b');
bodemag(quartercar({'body acceleration'; 'rattle space'; 'tire deflection'},'r') , 'b--');
hold on

% bodemag(quartercar_cl_hinf('body acceleration','r'),'r')
bodemag(quartercar_cl_hinf({'ab'; 'sd'; 'tire deflection'},'r') , 'k');
hold on

% bodemag(quartercar_cl_lqr('body acceleration','r'),'k')
bodemag(quartercar_cl_lqr({'body acceleration'; 'rattle space'; 'tire deflection'},'r'), 'r');

legend('Open-loop','hinf','LQR','location','SouthEast')
%% Time simulations
% % time and disturbance signals:
% t = 0:0.01:5;             % Time vector (5 seconds)
% r = zeros(size(t));       % Initialize road input
% r(t>=0.5 & t<=0.7) = 0.1; % 10cm bump between 0.5s and 0.7s
% 
% % input vectors:
% u_passive = [r; zeros(size(t))];
% u_active = [r; zeros(size(t))];
% 
% 
% % Run the simulation
% [y_pass, t_pass] = lsim(quartercar, u_passive, t);
% [y_act, t_act]   = lsim(quartercar_cl_lqr, u_passive, t);
% %% Plotting Results
% % figure;
% % title('Body Acceleration (m/s^2) - Passenger Comfort')
% % plot(t,y_pass(:,3),'r--');
% % hold on
% % plot(t, y_act(:,3),'b');
% % legend('Passive','LQR')
% % grid on
% % figure;
% bodemag(quartercar('ab','r'), 'r', quartercar_cl_lqr('ab','r'), 'b', {1, 100})
% legend('OPENLOOP','LQR','location','SouthEast')
% title('Gain from road dist (r) to body accel (ab) for OPEN-LOOP and LQR');
% % hold on
% % bodemag(sys_ol({'ab','sd'},'r'),'r',sys_ol({'ab','sd'},'fs'),'b',{1 100});
% % legend('Road disturbance (r)','Actuator force (fs)','location','SouthWest')
% % title(['Gain from road dist (r) and actuator force (fs)' newline ...
% %     'to body accel (ab) and suspension travel (sd)'])