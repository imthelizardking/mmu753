 function [hsum,Lkr,Lfin] = roadpr_ISO8608(Lkr,L1,L2,Gh0,k,N,q)

%  Generator of random profile defined by road profile vertical  
% - displacement PSD parameters (ISO 8608: 2016) unevenness index
% - (Gh0) and waviness (spectrum slope) (k):
%
%   Lkr -  sampling interval (m)
%   L1  -  minimal wavelength of generated profile [m],  
%   L2  -  maximum wavelength of generated profile [m], 
%   Gh0 -  PSD (one-sided) of the vertical road profile displacement
%          (i.e., displacement PSD) at reference angular frequency,
%          om0 = 1 rad/m; [m^3/rad]
%   k  -  waviness (slope of the straight line approximation of a raw PSD);
%   N  -  number of fourier coefficients;
%   q  -  number of non-overlapped signal blocks; 
%
%  Example of function calling in Matlab Command Window: 
%  Lkr = 0.1; L1 = 0.3534; L2 = 90.9;
%  Gh0 = 1e-6; k = 2; NFFT = 1024; q  = 4;
%  [hsum,Lkr,Lfin] = roadpr_ISO8608(Lkr,L1,L2,Gh0,k,NFFT,q);
%
% (c) Peter MUCKA, Institute of Materials and Machine Mechanics, Slovak Academy of Sciences - 03 Oct 2018

wmin = 2*pi/L2; wmax = 2*pi/L1; % Range of angular spatial frequency [rad/m] of artificial road profile
nmin = 1/L2;    nmax = 1/L1;    % Range of spatial frequency [1/m] of artificial  road profile

% disp(' '), disp(' Range of frequencies of generated road profile :'),
% disp([' Spatial frequency, n = ',num2str(nmin,3),' - ',num2str(nmax,3),' 1/m ;']),
% disp([' Angular spatial frequency, om = ',num2str(wmin,3),' - ',num2str(wmax,3),' rad/m;']),

NN = q*N; % number of profile samples

Lfin = (NN - 1)*Lkr; % total length (m) of generated random profile
L = 0:Lkr:Lfin; % distance vector
w = linspace(wmin,wmax,N)';  % equaly spaced vector of angular spatial frequency (wmin, wmax) of length N

deltaw = w(2) - w(1); % difference in angular spatial frequency
Gh = Gh0*w.^(-k); % Displacement PSD (m^3/rad)
fi0 = rand(N,1)*2*pi; % randomly distributed phase (0-2*pi)

W = w(:,ones(1,NN)); GH = Gh(:,ones(1,NN)); FI0 = fi0(:,ones(1,NN)); LL = L(ones(1,N),:);

% GENERATION OF ARTIFICIAL ROAD PROFILE IN RANGE OF WAVELENGTH: Lmin - Lmax

h = sqrt(2*GH.*deltaw).*cos(LL.*W + FI0);  
hsum = sum(h); % sum of signals h(wmin) + ... + h(wmax)

% generated road profile as a funcion of distance
% figure(1), clf,
% plot(L,hsum*100,'k','linewidth',3), 
% xlabel('{\itl} [m]','fontsize',40), ylabel('{\ith} [cm]','fontsize',44), set(gca,'xlim',[0 Lfin]),
return;
% CALCULATION OF DISPLACEMENT PSD

NOV = [ ]; % number of samples to overlap
dm = 'mean'; % detrending mode of signal

OMsmp = 2*pi/Lkr; Be = OMsmp/N;
PB = fix((length(h) - NOV)/(N - NOV)); % number of blocks

% disp([' Sampling interval, Lsmp  = ',num2str(Lkr),' m; Total length of profile, Ltotal = ',num2str(Lfin),' m;'])
% disp([' Block size for FFT = ',num2str(N),'; Number of signal samples to overlap = ',num2str(NOV),'; Number of blocks = ',num2str(PB),';'])
% disp([' Detrending mode = ',dm,';'])
% disp(' Profile windowing : Cosine Digital Tapering Window (CDTW)')


% Welch’s power spectral density estimate
% Using of Cosine Digital Tapering Window (CDTW) - prEN 13036-5: 2015
[Psigma,sigma] = psd_simple(hsum,N,1/Lkr,cdtw(N),NOV,dm); % Displacemnt PSD estimation as a function of wavenumber (1/m)

% Displacement PSD as a function of angular spatial frequency om (rad/m)
om = sigma*2*pi;                         om = om(om <= wmax); 
Pom = Psigma./(2*pi*nmax*(L1/(2*Lkr)));  Pom = Pom(1:length(om));
RMS_h_FR = sqrt(trapz(om,Pom))*1e3; % RMS value of generated road profile (frequency domain)

 
% Estimation of PSD parameters -  Gh0sim, ksim
om(1) = []; Pom(1) = [];  % Ignore the first point of displacement PSD
  
Lmin = 0.3534; Lmax = 90.9091; % Wavelength range for PSD fitting according to the ISO 8608: 2016
% Lmin = 0.78125; Lmax = 50; disp(' Warning !!! Change of fitting wavelength interval L1-L2 z .35 m - 90.9 m na 0.78 - 50 m !!!') 
   
ii = find(om >= (1/Lmax)*2*pi & om <= (1/Lmin)*2*pi); % ii - samples from interval Lmin - Lmax 
omint = linspace(2*pi/Lmax,2*pi/Lmin,400); Pomint = interp1(om,Pom,omint,'linear','extrap'); % data interpolation
pp = polyfit(log10(omint),log10(Pomint),1);  % fitting the non-smoothed displacement PSD by a straightline
Gh0sim = 10^(pp(2)); ksim = -pp(1); % Unevenness index and waviness of fitted PSD

% % Displacement PSD - visualisation
% figure(2), clf,
% loglog(om,Pom,'k','linewidth',2), hold on, set(gca,'xtick',[.02 .05 .1 .2 .5 1 2 5 10 20 50],'ytick',10.^(-20:0),'fontsize',36,'linewidth',2), axis square, 
% hold on, axis([0.85*min(om) 1.1*max(om) 10^floor(log10(min(Pom))) 10^ceil(log10(max(Pom)))]),
% loglog(om(ii),Gh0sim*10^(-6)*om(ii).^(-ksim),'k','linewidth',4), 
% legend('Non-smoothed PSD','Fitted PSD')
% xlabel('{\Omega} [rad/m]','fontsize',38), ylabel('{\itG}_H({\Omega}) [m^3/rad]','fontsize',40),

% disp([' Fitting interval of non-smoothed displacement PSD: Lmin - Lmax = ',num2str(Lmin),' - ',num2str(Lmax),' m;']),
% disp(' ')
% disp(' Road unevennes PSD parameters (TARGET/GENERATED):'), 
% disp([' Unevenness index, Gh(om0) = ',num2str(Gh0*1e6,3),' x 10^-6 m^3/rad; Waviness, w = ',num2str(k,3),'; (TARGET)']),
% disp([' Unevenness index, Gh(om0) = ',num2str(Gh0sim*1e6,4),' x 10^-6 m^3/rad; Waviness, w = ',num2str(ksim,3),' (GENERATED);']),


% Check of signal RMS values in frequency domain and in distance domain

RMS_h_DIST = std(hsum)*1e3; % RMS value of vertical displacement (distance domain)

% disp(' Root mean square (RMS) of the generated road profile :') 
% disp([' RMS{h} = ',num2str(RMS_h_FR,4),' mm (frequency domain)']) 
% disp([' RMS{h} = ',num2str(RMS_h_DIST,4),' mm (distance domain)']) 




