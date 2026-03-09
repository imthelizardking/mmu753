function [A_cl, K] = fun_controller_quartercar_activesuspension_hinf(A,B,C,D)
W_road = 0.1; % 10cm bumps
Wroad.u = 'd1';   Wroad.y = 'r'; % d1 will be actual road disturbance model
Wact = 0.8*tf([1 50],[1 500]);  Wact.u = 'u';  Wact.y = 'e1';
Wd2 = ss(0.01);  Wd2.u = 'd2';   Wd2.y = 'Wd2'; % sensor noise for body acc
Wd3 = ss(0.5);   Wd3.u = 'd3';   Wd3.y = 'Wd3'; % sensor noise for suspension travel