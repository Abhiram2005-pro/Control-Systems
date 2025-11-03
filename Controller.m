%% ========================================
%   MPC Design & Simulation for SRM Motor
%   ========================================
clc; clear; close all;

%% Motor Parameters (example values — replace with actual SRM values)
R = 0.5;        % Phase resistance (Ohm)
L = 0.01;       % Phase inductance (H)
J = 0.001;      % Rotor inertia (kg.m^2)
B = 0.0001;     % Friction coefficient (N.m.s)
K = 0.05;       % Torque constant (Nm/A)

%% Continuous-Time Plant
s = tf('s');
Plant = K / ((L*s + R)*(J*s + B));  % Simplified current-to-speed model
Plant_ss = ss(Plant);

%% Discretization
Ts = 1e-3;                     % Sampling time
Plant_d = c2d(Plant_ss, Ts);   % Discrete model

% Extract matrices
Ad = Plant_d.A;
Bd = Plant_d.B;
Cd = Plant_d.C;
Dd = Plant_d.D;

%% MPC Parameters
PredictionHorizon = 20;
ControlHorizon    = 5;

mpcobj = mpc(Plant_d, Ts, PredictionHorizon, ControlHorizon);

%% Constraints
mpcobj.MV.Min = -5;
mpcobj.MV.Max =  5;

%% Weights
mpcobj.Weights.OutputVariables          = 1;
mpcobj.Weights.ManipulatedVariables     = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;

%% Initialize states
x = zeros(size(Ad,1),1);   % plant state
xmpc = mpcstate(mpcobj);   % mpc state

%% Simulation Setup
Nsim = 50;
ref  = ones(Nsim,1);   % step reference
y    = zeros(Nsim,1);
duty_cycle    = zeros(Nsim,1);

%% Closed-loop Simulation
disp('Running closed-loop MPC simulation...');

for k = 1:Nsim
    % Compute control input from MPC
    duty_cycle(k) = mpcmove(mpcobj, xmpc, y(max(k-1,1)), ref(k));

    % Plant update (discrete-time state-space)
    x = Ad*x + Bd*duty_cycle(k);
    y(k) = Cd*x + Dd*duty_cycle(k);
end

%% Plot results
figure;
subplot(2,1,1);
plot((0:Nsim-1)*Ts, y, 'b','LineWidth',1.5);
title('MPC Output Response'); ylabel('Output (Torque/Speed)');
grid on;

subplot(2,1,2);
stairs((0:Nsim-1)*Ts, duty_cycle, 'r','LineWidth',1.5);
title('MPC Control Input'); ylabel('Input (Voltage Command)');
xlabel('Time (s)'); grid on;

disp('✅ Simulation complete. Use "mpcobj" in Simulink MPC Controller block.');
