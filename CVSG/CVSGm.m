%% Conventional Virtual Synchrnous Generator control scheme.
% Quang-Manh Hoang, Ph.D. Student at University of Michigan - Dearborn
% Settling time: 0.5s, without overshoot
% Last update: 06/29/2025.
%% -------------------------------------
clear all; clc;
% Define Simulink sampling time
params.Tsam = 1e-5;
% Load frequency
params.f0=50;
%% LCL - filter design
% DC link voltage
params.Vdc=800;
%Switching frequency
params.fs=10e3;
params.Ts=1/params.fs;
params.Filter.Ls_1=1e-3;
params.Filter.Cs = 50e-6;
params.Vn   = 110;  % 
params.wn = 2*pi*50;
params.Vj = 110;
params.XR = 1;
%% PI current controller-----------
params.Tresc=1e-3;
params.e=1;
params.omega=2*pi/params.Tresc;
params.k=1/params.Filter.Ls_1;
params.Kpc=2*params.e*params.omega/params.k;
params.Kic=params.omega^2/params.k;

%% PI voltage controller-----------
params.Tresv=10e-3;
params.ev=1;
params.omegav=2*pi/params.Tresv;
params.kv=1/params.Filter.Cs;
params.Kpv=2*params.ev*params.omegav/params.kv;
params.Kiv=params.omegav^2/params.kv;
%% Power control loop-----------
params.Pref = 4.5e3;         % 
params.Qref = 1000;         % 
% 3a) Conventional P-loop (fixed gains)
Kp = 0.007666307989764;
Dp = 2.087054162363824e+03;

% 3b) Conventional Q-loop (fixed gains)
Kq = 0.115300461672244;
Dq = 0.686969675951841;