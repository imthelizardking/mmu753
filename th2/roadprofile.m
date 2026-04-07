% clc; clear; close all;

u0 = 5; % Velocity

%% Road Profile

Lkr = 0.01; L1 = 0.3534; L2 = 90.9; NFFT = 1024; q  = 10;

% Gh0 = 1*(1e-6); kroad = 2; %Type A
% Gh0 = 4*(1e-6); kroad = 2; %Type B
% Gh0 = 16*(1e-6); kroad = 2; %Type C
% Gh0 = 64*(1e-6); kroad = 2; %Type D
% Gh0 = 256*(1e-6); kroad = 2; %Type E
% Gh0 = 1024*(1e-6); kroad = 2; %Type F
% Gh0 = 4096*(1e-6); kroad = 2; %Type G
% Gh0 = 16384*(1e-6); kroad = 2; %Type H

% Gh0 = (4.3e-11)/(2*pi); kroad = 2; %Smooth airfield runway
% Gh0 = (8.1e-6)/(2*pi); kroad = 2; %Rough airfield runway
% Gh0 = (6.45e-8)/(2*pi); kroad = 2; %Tarmac motorway
% Gh0 = (2.14e-7)/(2*pi); kroad = 2; %Concrete motorway
% Gh0 = (3.0e-7)/(2*pi); kroad = 2; %Good road
% Gh0 = (4.8e-7)/(2*pi); kroad = 2; %Smooth highway
% Gh0 = (2.0e-6)/(2*pi); kroad = 2; %Average road
% Gh0 = (4.39e-6)/(2*pi); kroad = 2; %Minor road
% Gh0 = (4.4e-6)/(2*pi); kroad = 2; %Highway with gravel
% Gh0 = (1.5e-5)/(2*pi); kroad = 2; %Poor road
% Gh0 = (5.17e-5)/(2*pi); kroad = 2; %Very rough unmade road
% 
% Gh0 = (1.2e-5)/(2*pi); kroad = 2; %Pasture
% Gh0 = (3.16e-5)/(2*pi); kroad = 2; %Cross country medium roughness
% Gh0 = (4.98e-5)/(2*pi); kroad = 2; %Grassland
 Gh0 = (2.34e-4)/(2*pi); kroad = 2; %Rocky Soil
% Gh0 = (3.6e-4)/(2*pi); kroad = 2; %Cross country severe
% 
% Gh0 = (3.44e-4)/(2*pi); kroad = 2; %Random Test Course
% Gh0 = (1.01e-3)/(2*pi); kroad = 2; %Rocky Test Course

[hsum,Lkr,Lfin] = roadpr_ISO8608(Lkr,L1,L2,Gh0,kroad,NFFT,q);
ts = timeseries(hsum); %plot(ts)
t = 0:(Lfin/u0)/(length(hsum)-1):(Lfin/u0);
% x = t*u0; % new_hsum = hsum(x);
road_sig = timeseries(hsum(1:length(t)),t); %plot(road_sig)