% Ex10_1.m
% Drive cycle and driving power calculation
clear all, close all
load CYC_HWFET.mat % Load highway cycle
time_highway = cyc_mph(:,1);
speed_highway = cyc_mph(:,2);
figure(1), subplot(221)
plot(time_highway, speed_highway)
xlabel('Time (Sec)')
ylabel('Speed (mph)')
title('EPA highway cycle')