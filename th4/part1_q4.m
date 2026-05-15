clf; clear all; clc;
%%
m = 1250;
ro = 1.225;
A = 1.2;
Cd = 0.5;
f = 0.015;
R = 3;
I_e = 0.15;
r = 0.3;
I_w = 1.0;
u_0 = 40;
u_w = 2;
g = 9.81;
theta_0 = 0;
%% create sys and bodes for different u_0
u_0s = [20;30;37;48;73];
I_w_total = 4 * I_w;
s = tf('s');
for i=1:size(u_0s,1)
    u_0 = u_0s(i);
    if u_0<=25 % gear 2
        G = 2.2;
    elseif u_0<=35 % gear 3
        G = 1.5;
    elseif u_0<=45 % gear 4
        G = 1.1;
    elseif u_0<=55 % gear 5
        G = 0.85;
    end
    m_eqv = m + I_w_total/r^2 + I_e*(G*R)^2/r^2;

    F_x0 = m*g*sin(theta_0) + f*m*g*cos(theta_0) + ...
        0.5*ro*A*Cd*(u_0+u_w)^2;

    tau(i) = m_eqv/(ro*A*Cd*(u_0+u_w));
    K(i) = 1/(ro*A*Cd*(u_0+u_w));
    d(i) = m*g*(-cosd(theta_0)+f*sind(theta_0));
    sys_lin(:,:,i) = K(i)/(tau(i)*s+1);
    figure(1);
    bode(sys_lin(:,:,i));
    hold on;
end
legend_labels = string(u_0s);
legend(legend_labels);