clf; clear all; clc;
%%
a=1.14;
L=2.54;
m=1500;
Iz=2420;
Caf=44000*2;
Car=47000*2;
b=L-a;
g= 9.81;
u_0 = 15;

K_r = (m*u_0^2*a-b*(a+b)*Car)/...
      (m*u_0^2*b-a*(a+b)*Caf)*...
       Caf/Car;