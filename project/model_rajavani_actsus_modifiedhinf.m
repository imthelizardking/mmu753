%% active suspension model used in rajavani for LQR control extended for h-infinity synthesis
% states:
%   x1 = zs-zu (rattle space)
%   x2 = zs_dot (body velocity)
%   x3 = zu-zr (tire displacement)
%   x4 = zu_dot (tire velocity)
% inputs:
%   u: actuator force
% outputs:
%   y1 = zs_dot_dot (body acceleration)
%   y2 = zs-zu (=x1) (rattle space)
% virtual outputs:
%   z1 = zs_dot_dot (body acceleration)
%   z2 = zs-zu (=x1) (rattle space)
%%
% % Physical parameters (taken from bd example)
% ms = 300;    % kg
% mu = 60;     % kg
% bs = 1000;   % N/m/s
% ks = 16000 ; % N/m
% kt = 190000; % N/m
% bt = 0;      % N/m/s
% Physical parameters (taken from 1989 br)
ms = 240;    % kg
mu = 36;     % kg
bs = 1000;   % N/m/s
ks = 16000 ; % N/m
kt = 160000; % N/m
bt = 0;      % N/m/s
% system dynamics
A = [0, 1, 0, -1;...
     -ks/ms, -bs/ms, 0, bs/ms;...
     0, 0, 0, 1;...
     ks/mu, bs/mu, -kt/mu, -(bs+bt)/mu];

B = [0;...
     1/ms;...
     0;...
     -1/mu];

L = [0;...
     0;...
     -1;...
     0];

B_x = [B,L];

B1 = L;
B2 = B;

C = [A(2,:);...
     A(1,:);...
     A(3,:)];

C1 = C;
C2 = C;

D11 = [0;...
       0;...
       0];
D12 = [1/ms;...
       0;...
       0];
D21 = [0;...
       0;...
       0];
D22 = [1/ms;...
       0;...
       0];
D = [D22, D21];
quartercar = ss(A, B_x, C, D);
quartercar.StateName = {'rattle space'; 'body velocity'; 'tire deflection'; 'tire velocity'};
quartercar.InputName = {'fs'; 'r'};
quartercar.OutputName = {'body acceleration'; 'rattle space'; 'tire deflection'};

quartercar_x = ss(A, B_x, C, D);
quartercar_x.StateName = {'rattle space'; 'body velocity'; 'tire deflection'; 'tire velocity'};
quartercar_x.InputName = {'fs'; 'r'};
quartercar_x.OutputName = {'ab'; 'sd'; 'tire deflection'};

% quartercar_x = ss(A,[B1, B2],[C1; C2],[D11, D12; D21, D22]);
% quartercar_x.StateName = {'rattle space';'body velocity'; 'tire deflection';'tire velocity'};
% quartercar_x.InputName = {'r', 'fs'};
% quartercar_x.OutputName = {'body acceleration';'rattle space'; 'tire deflection'; 'e1'; 'e2'; 'e3'};