%% ANN-based GIE Adaptive Gain-Scheduling Control Scheme for VSG.
% Quang-Manh Hoang, Ph.D. Student at University of Michigan - Dearborn
% Settling time: 1s, without overshoot
% Last update: 06/29/2025.
clear all; clc;
load("ann_model_params.mat")
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
params.Filter.Ls_2=0.5e-3;
params.Filter.Cs = 50e-6;
params.Pref = 4.5e3;         % 
params.Qref = 1000;         % 
params.Vn   = 110;  % 
params.wn = 2*pi*50;
params.Vj = 110;
params.XR = 1;
%% PI current-----------
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
%% Power control loop-----------------

% Initial values
Kp = 0.007666307989764;
Dp = 2.087054162363824e+03;

Kq = 0.115300461672244;
Dq = 0.686969675951841;
%% 2. Load model
mdl     = "AdaptiveCVSG";
gainDp = 'AdaptiveCVSG/Dp';
gainKp = 'AdaptiveCVSG/Kp';
gainDq = 'AdaptiveCVSG/Dq';
gainKq = 'AdaptiveCVSG/Kq';
gainK11 = 'AdaptiveCVSG/KPQ11';
gainK12 = 'AdaptiveCVSG/KPQ12';
gainK21 = 'AdaptiveCVSG/KPQ21';
gainK22 = 'AdaptiveCVSG/KPQ22';

load_system(mdl);
set_param(mdl, "StopTime", "60");    % simulation time 30 s
set_param(mdl, "SimulationCommand", "start");
disp("Model started ...");

%% 3. Monitoring iteration and Controller gains update
while strcmp(get_param(mdl,'SimulationStatus'),'running')
    pause(0.5);  
    runObj = [];
    while isempty(runObj)
        runObj = Simulink.sdi.getCurrentSimulationRun(mdl);
        pause(0.01);
    end

    % 2) wait until the first signal has at least one sample
    id1 = runObj.getSignalIDByIndex(1);
    id2 = runObj.getSignalIDByIndex(2);
    sig1 = Simulink.sdi.getSignal(id1);
    sig2 = Simulink.sdi.getSignal(id2);
    while isempty(sig1.Values.Data) && isempty(sig2.Values.Data)
        pause(0.01);
        sig1 = Simulink.sdi.getSignal(id1);
        sig2 = Simulink.sdi.getSignal(id2);
    end
    tsRg = sig1.Values;
    tsLg = sig2.Values;
    Rg = tsRg.Data(end);
    Lg = tsLg.Data(end);
    fprintf("Lg = %f, Rg = %f\n", Lg, Rg);
    % Update gains
    [Kp, Dp, Kq, Dq, K11, K12, K21, K22] = compute_gain_Lg_Rg(Lg, Rg, params);
    
    set_param(gainDp, 'Gain', mat2str(Dp));
    set_param(gainKp, 'Gain', mat2str(Kp));
    set_param(gainDq, 'Gain', mat2str(Dq));
    set_param(gainKq, 'Gain', mat2str(Kq));
    set_param(gainK11, 'Gain', mat2str(K11));
    set_param(gainK12, 'Gain', mat2str(K12));
    set_param(gainK21, 'Gain', mat2str(K21));
    set_param(gainK22, 'Gain', mat2str(K22));
    
end

disp("Simulation finished.");

%% --- Local function: Recalculate controller gains function ---
function [Kp, Dp, Kq, Dq, K11, K12, K21, K22] = compute_gain_Lg_Rg(Lg, Rg, params)
% Unpack
f0   = params.f0;
Pref = params.Pref;
Qref = params.Qref;
Vn   = params.Vn;
Vj   = params.Vj;
Xg = 2*pi*f0*Lg;

% Finding operating points
[Ess, delta_ss] = solve_E_delta(Pref, Qref, Vn, Rg, Lg, f0);
Vi0    = Ess;
theta0 = delta_ss;

% Update Jacobian matrix
D = Rg^2 + Xg^2;
K11 = (3/D)*( Rg*Vi0*Vj*sin(theta0) + Xg*Vi0*Vj*cos(theta0) );
K12 = (3/D)*( Rg*(2*Vi0 - Vj*cos(theta0)) + Xg*Vj*sin(theta0) );
K21 = (3/D)*( Xg*Vi0*Vj*sin(theta0) - Rg*Vi0*Vj*cos(theta0) );
K22 = (3/D)*( Xg*(2*Vi0 - Vj*cos(theta0)) - Rg*Vj*sin(theta0) );
KPQ = [K11 K12; K21 K22];

% Calculate new gains
Kp = 16 / KPQ(1,1);
Dp = KPQ(1,1) / 2;
Dq = KPQ(2,2) / 100;
Kq = 4 / ( KPQ(2,2) + Dq );
end

%% --- Local function: Finding operating points ---
function [Ess, delta_ss] = solve_E_delta(Pref, Qref, Vn, Rg, Lg, f)
Xg = 2*pi*f*Lg;
D  = Rg^2 + Xg^2;
k  = 3 / D;
fun = @(x)[ ...
    Pref - k*x(1)*( Rg*(x(1) - Vn*cos(x(2))) + Vn*Xg*sin(x(2)) );
    Qref - k*x(1)*( Xg*(x(1) - Vn*cos(x(2))) - Vn*Rg*sin(x(2)) ) ];

Pslope = Pref*D / (3*Vn^2);
delta0 = asin( max(min(Pslope,0.99),-0.99) );
x0     = [Vn, delta0];

opts = optimoptions('fsolve','Display','off', ...
    'FunctionTolerance',1e-12,'StepTolerance',1e-12);
sol  = fsolve(fun, x0, opts);
Ess      = sol(1);
delta_ss = sol(2);
end
