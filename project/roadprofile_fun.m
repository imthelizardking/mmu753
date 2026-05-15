function [road_sig, t, hsum] = roadprofile_fun(road_type, u0)
% GENERATE_ROAD_PROFILE generates a road disturbance timeseries
%
% Inputs:
%   road_type : string, e.g. 'Smooth highway', 'Average road', etc.
%   u0        : vehicle velocity in m/s
%
% Output:
%   road_sig  : MATLAB timeseries of road profile
%
% Supported road_type values:
%   ISO Classes:     'Type A' through 'Type H'
%   Paved roads:     'Smooth highway', 'Average road', 'Minor road',
%                    'Highway with gravel', 'Poor road',
%                    'Very rough unmade road', 'Tarmac motorway',
%                    'Concrete motorway', 'Good road'
%   Airfield:        'Smooth airfield runway', 'Rough airfield runway'
%   Off-road:        'Pasture', 'Cross country medium roughness',
%                    'Grassland', 'Rocky Soil', 'Cross country severe'
%   Test courses:    'Random Test Course', 'Rocky Test Course'

%% Fixed parameters
Lkr  = 0.01;
L1   = 0.3534;
L2   = 90.9;
NFFT = 1024;
q    = 100;
kroad = 2;

%% Select Gh0 based on road type
switch road_type
    % ── ISO 8608 Standard Classes ──────────────────────────────────────
    case 'Type A',  Gh0 = 1    * 1e-6;
    case 'Type B',  Gh0 = 4    * 1e-6;
    case 'Type C',  Gh0 = 16   * 1e-6;
    case 'Type D',  Gh0 = 64   * 1e-6;
    case 'Type E',  Gh0 = 256  * 1e-6;
    case 'Type F',  Gh0 = 1024 * 1e-6;
    case 'Type G',  Gh0 = 4096 * 1e-6;
    case 'Type H',  Gh0 = 16384* 1e-6;

    % ── Airfield ───────────────────────────────────────────────────────
    case 'Smooth airfield runway', Gh0 = (4.3e-11)  / (2*pi);
    case 'Rough airfield runway',  Gh0 = (8.1e-6)   / (2*pi);

    % ── Paved Roads ────────────────────────────────────────────────────
    case 'Tarmac motorway',        Gh0 = (6.45e-8)  / (2*pi);
    case 'Concrete motorway',      Gh0 = (2.14e-7)  / (2*pi);
    case 'Good road',              Gh0 = (3.0e-7)   / (2*pi);
    case 'Smooth highway',         Gh0 = (4.8e-7)   / (2*pi);
    case 'Average road',           Gh0 = (2.0e-6)   / (2*pi);
    case 'Minor road',             Gh0 = (4.39e-6)  / (2*pi);
    case 'Highway with gravel',    Gh0 = (4.4e-6)   / (2*pi);
    case 'Poor road',              Gh0 = (1.5e-5)   / (2*pi);
    case 'Very rough unmade road', Gh0 = (5.17e-5)  / (2*pi);

    % ── Off-road ───────────────────────────────────────────────────────
    case 'Pasture',                        Gh0 = (1.2e-5)  / (2*pi);
    case 'Cross country medium roughness', Gh0 = (3.16e-5) / (2*pi);
    case 'Grassland',                      Gh0 = (4.98e-5) / (2*pi);
    case 'Rocky Soil',                     Gh0 = (2.34e-4) / (2*pi);
    case 'Cross country severe',           Gh0 = (3.6e-4)  / (2*pi);

    % ── Test Courses ───────────────────────────────────────────────────
    case 'Random Test Course', Gh0 = (3.44e-4) / (2*pi);
    case 'Rocky Test Course',  Gh0 = (1.01e-3) / (2*pi);

    otherwise
        error('Unknown road type: "%s"', road_type);
end

%% Generate road profile
[hsum, ~, Lfin] = roadpr_ISO8608(Lkr, L1, L2, Gh0, kroad, NFFT, q);

%% Build timeseries
t        = 0 : (Lfin/u0)/(length(hsum)-1) : (Lfin/u0);
road_sig = timeseries(hsum(1:length(t)), t);

end