%% srm_mpc_setup.m
clc; clear; close all;

%% Motor & sample time
J   = 0.01;      % kg·m^2
Bf  = 0.001;     % N·m·s   (friction, renamed from B)
Kt  = 0.20;      % N·m/A
L   = 0.05;      % H
Rs  = 1.0;       % Ohm
Ts  = 1e-3;      % s  (MPC sample time)

%% Continuous-time plant: x=[i; w], u=[V; TL], y=[w]
A = [-Rs/L      0;
      Kt/J   -Bf/J];

Bmat = [ 1/L   0;      % input #1: V  (MV)
         0   -1/J];    % input #2: TL (MD)

C = [0 1];             % measure speed only  <-- scalar output
D = [0 0];

sys_c = ss(A,Bmat,C,D);

%% Discretize
sysd = c2d(sys_c, Ts);

%% Tell MPC which signal is which
% Input1 = MV (voltage), Input2 = MD (load torque), Output1 = MO (speed)
sysd = setmpcsignals(sysd, 'MV',1, 'MD',2, 'MO',1);

%% MPC object
PredictionHorizon = 20;
ControlHorizon    = 5;
mpcobj = mpc(sysd, Ts, PredictionHorizon, ControlHorizon);

%% Constraints (tune to your rig)
mpcobj.MV.Min = -50;          % V
mpcobj.MV.Max =  50;          % V
mpcobj.OV.Min = -300;         % rad/s
mpcobj.OV.Max =  300;         % rad/s

%% Weights (start conservative; tune later)
mpcobj.Weights.ManipulatedVariablesRate = 0.2;
mpcobj.Weights.ManipulatedVariables     = 0.05;
mpcobj.Weights.OutputVariables          = 1.0;
mpcobj.Weights.ECR                      = 1e5;

%% Disturbance model: integral action on output ⇒ offset-free tracking
setoutdist(mpcobj,'integrators');

%% (Optional) basic noise models
%mpcobj.Model.Plant.Noise = 1e-4;  % white noise on measured output

%% Aliases for Simulink blocks that may still expect A,B,C,D
%A = A; %#ok<NASGU>
B = Bmat;  %#ok<NASGU>   % <--- alias so State-Space/Gain blocks find "B"
%C = C; %#ok<NASGU>
%D = D; %#ok<NASGU>

assignin('base','A',A);
assignin('base','B',B);       % alias
assignin('base','Bmat',Bmat); % proper name
assignin('base','C',C);
assignin('base','D',D);
assignin('base','sys',sys_c);
assignin('base','sysd',sysd);
assignin('base','mpcobj',mpcobj);
assignin('base','Ts',Ts);
assignin('base','J',J);
assignin('base','Bf',Bf);
assignin('base','Kt',Kt);
assignin('base','L',L);
assignin('base','Rs',Rs);

disp('✅ MPC object (mpcobj) and plant (sysd) are in the base workspace.');
disp('   MPC mo input is SCALAR (speed). MV=V, MD=load torque.');
