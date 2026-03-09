%%
% brian douglas example
% states: body travel, body velo, wheel travel, wheel vel
% inputs: disturbance (r), actuator force (fs)
% outputs: body travel (xb), suspension travel (sd), body acc (ab)
%% %%%%%%%%%%%%%%%%%%%
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