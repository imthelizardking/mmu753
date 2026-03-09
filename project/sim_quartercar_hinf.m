close all; clear all; clc;
%%
%
%
%%
% Physical parameters
mb = 300;    % kg
mw = 60;     % kg
bs = 1000;   % N/m/s
ks = 16000 ; % N/m
kt = 190000; % N/m

% State matrices
A = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
      0 0 0 1; [ks bs -ks-kt -bs]/mw];
B = [ 0 0; 0 1e3/mb ; 0 0 ; [kt -1e3]/mw];
C = [1 0 0 0; 1 0 -1 0; A(2,:)];
D = [0 0; 0 0; B(2,:)];

quartercar = ss(A,B,C,D);
quartercar.StateName = {'body travel (m)';'body vel (m/s)';...
          'wheel travel (m)';'wheel vel (m/s)'};
quartercar.InputName = {'r';'fs'};
quartercar.OutputName = {'xb';'sd';'ab'};

model_actuator_ = tf(1,[1/60 1]); % actuator model
[A_, B_, C_, D_] = tf2ss(cell2mat(model_actuator_.num), cell2mat(model_actuator_.den));
model_actuator = ss(1);
model_actuator.A = A_; model_actuator.B = B_; model_actuator.C = C_; model_actuator.D = D_;
model_actuator.InputName = 'u';
model_actuator.OutputName = 'fs';

% output weighting filters
Wact = 0.8*tf([1 50],[1 500]);  Wact.u = 'u';  Wact.y = 'e1'; % actuator filter to pass hf signals
Wsd_template = 0.04 * tf([1/8 1],[1/80 1]);
Wab_template = 0.4 * tf([1/0.45 1],[1/150 1]);
beta = 0.5;
Wsd = beta / Wsd_template; Wsd.u = 'sd';  Wsd.y = 'e3';
Wab = (1-beta) / Wab_template; Wab.u = 'ab';  Wab.y = 'e2';

Wroad = ss(0.07);  Wroad.u = 'd1';   Wroad.y = 'r';
Wd2 = ss(0.01);  Wd2.u = 'd2';   Wd2.y = 'Wd2';
Wd3 = ss(0.5);   Wd3.u = 'd3';   Wd3.y = 'Wd3';
y1_ = sumblk('y1 = sd + Wd2'); y2_ = sumblk('y2 = ab + Wd3');
quartercar_scheme = connect(quartercar(2:3,:), Wroad, model_actuator, Wact, Wab, Wsd, Wd2, Wd3, y1_, y2_, {'d1', 'd2', 'd3', 'fs'}, {'e1', 'e2', 'e3', 'y1', 'y2'});

% design hinf controller
ncont = 1; % 1 control signal u
nmeas = 2; % 2 measurements, sd and ab
[K,~,gamma] = hinfsyn(quartercar_scheme,nmeas,ncont);

% closed-loop model
K.u = {'sd','ab'};  K.y = 'u';
CL = connect(quartercar, model_actuator, K,'r',{'xb';'sd';'ab'});

% Road disturbance
t = 0:0.0025:1;
roaddist = zeros(size(t));
roaddist(1:101) = 0.025*(1-cos(8*pi*t(1:101)));

% Closed-loop model
SIMK = connect(quartercar,model_actuator,K,'r',{'xb';'sd';'ab';'fs'});

% Simulate
p1 = lsim(quartercar(:,1),roaddist,t);
y = lsim(SIMK(1:4,1),roaddist,t);

subplot(211)
plot(t,p1(:,1),'b',t,y(:,1),'m',t,roaddist,'g')
title('Body travel'), ylabel('x_b (m)')
subplot(212)
plot(t,p1(:,3),'b',t,y(:,3),'m',t,roaddist,'g')
title('Body acceleration'), ylabel('a_b (m/s^2)')
legend