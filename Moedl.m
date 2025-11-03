clc; clear; close all;

%% ========================
%  Motor Parameters (SRM example)
%  ========================
J  = 0.01;    % Inertia (kg·m^2)
Bf = 0.001;   % Friction coefficient (N·m·s)  (renamed from "B")
Kt = 0.2;     % Torque constant (N·m/A)
L  = 0.05;    % Inductance (H)
Rs = 1;       % Stator resistance (Ohm)
Ts = 1e-3;    % Sampling time (s)

%% ========================
%  Step 1: Define Continuous Plant Model
%  States: [i; w], Inputs: [V; TL], Output: [w]
%  ========================
A = [-Rs/L      0;
      Kt/J   -Bf/J];

Bmat = [1/L    0;      % u input (control voltage, MV)
        0    -1/J];    % TL input (load torque, MD)

C = [0 1];             % ✅ output = speed only (scalar)
D = [0 0];

sys = ss(A,Bmat,[0 1],0);   % only ω output
sysd = c2d(sys, Ts);
sysd = setmpcsignals(sysd,'MV',1,'MD',2,'MO',1);


%% ========================
%  Step 2: Discretize the Plant
%  ========================
sysd = c2d(sys, Ts);

%% ========================
%  Step 3: Assign Input/Output Roles
%  ========================
% Input1 = MV (control voltage)
% Input2 = MD (load torque disturbance)
% Output1 = MO (speed)
sysd = setmpcsignals(sysd, 'MV',1,'MD',2,'MO',1);

%% ========================
%  Step 4: Create MPC Controller
%  ========================
PredictionHorizon = 10;
ControlHorizon    = 3;

mpcobj = mpc(sysd, Ts, PredictionHorizon, ControlHorizon);

%% ========================
%  Step 5: Constraints
%  ========================
mpcobj.MV.Min = -50;   % Voltage lower bound
mpcobj.MV.Max =  50;   % Voltage upper bound
mpcobj.OV.Min = -200;  % Speed lower bound
mpcobj.OV.Max =  200;  % Speed upper bound

%% ========================
%  Step 6: Weights
%  ========================
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
mpcobj.Weights.ManipulatedVariables     = 0.1;
mpcobj.Weights.OutputVariables          = 1;
mpcobj.Weights.ECR = 1e5;

%% ========================
%  Step 7: Disturbance Model
%  ========================
setoutdist(mpcobj, 'integrators');

%% ========================
%  Done
%  ========================
disp('✅ MPC object created successfully. Ready for Simulink.');
B = Bmat;   % alias for Simulink blocks expecting "B"
