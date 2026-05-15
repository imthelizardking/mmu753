clear all; clf; clc;
%% Ex12_1.m 
% Define the linearized model parameters 
u0=20; g=9.81; m=1000; f=0.015; Theta=0; rho=1.202; 
A=1; Cd=0.5; uw=2; 
% Calculate the equilibrium force, Fx0: 
Fx0 = m*g*sin(Theta) + f*m*g + ... 
    0.5*rho*A*Cd*(u0+uw)^2; 
% The time constant and dc gain are: 
Tau=(m/(rho*A*Cd*(u0+uw)));
K=Tau/m; 
% Fix the Ti gain (Ki=Kp/Ti) for the PI control: 
Ti=186.86/10.0;
% % The loop transfer function for PI control is % Kp*(num/den) where num/dec = (1+1/Ti*s)*K/(1+Tau*s)
% num_o=K*[Ti 1]; den_o=[Ti*Tau Ti 0];
% % Calculate open loop zeros and poles:
% z=roots(num_o); p=roots(den_o);
% % Obtain the root locus for Kp:
% K_root=[0:10:300];
% R=rlocus(num_o,den_o,K_root);
% plot(R,'*'); title('RootLocus Plot'); axis([-0.2 0 -0.05 0.05]);
% pause;
% [K_root, POLES]=rlocfind(num_o,den_o)  % Simulate the reference unit step response % with and w/o control:
t=[0:0.5:100];
sys = tf([K],[Tau 1]);
y=step(sys,t);

Kp=186.86; KI = Kp/Ti;
numc=K*[Kp KI]; denc=[Tau K*Kp+1 K*KI];
% Calculate the closed-loop zeros and poles:
zc=roots(numc)
pc=roots(denc)
sys_c = tf(numc, denc);
%%
yc=step(sys_c, t);
plot(t,y,'-r',t,yc,'--b'); 
grid;
xlabel('Time (sec)')
title('Unit Step Reference Response');
legend('Open-loop', 'Closed-loop');
pause;
%%
% Simulate the disturbance unit step response % with and w/o control:
yd=step(sys,t); sys_disturbance = tf([K 0], denc); ycd=step(sys_disturbance,t);
plot(t,yd,'--r',t,ycd,'--b');
xlabel('Time (sec)'); grid;
title('Unit Step Disturbance Response'); 
legend('Open-loop', 'Closed-loop');