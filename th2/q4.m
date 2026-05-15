clf; clear all; clc;
%%
init_params;
%%
s = tf('s');
H1 =  s*(c*s+k)/...
     (m*s^2+c*s+k); % transfer function from road height speed to sprung acceleration
H2 = -(m*s)/...
     (m*s^2+c*s+k); % transfer function from road height speed to rattle space displacement
bode(H1);
title('H1');
figure
bode(H2);
title('H2');