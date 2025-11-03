clc;
clear;
close all;

%% ========================
%  Motor Parameters (Simplified SRM)
%  ========================
J = 0.01; 
B = 0.001;
Kt = 0.2;
L = 0.05;
Rs = 1;
Ts = 0.001;   % Sampling time

%% ========================
%  MPC Parameters
%  ========================
Np = 5;          % Prediction horizon
Q = 10;          % Weight on speed error
R_weight = 0.1;  % Weight on control effort

%% ========================
%  Simulation Setup
%  ========================
t_end = 2;               % seconds
time = 0:Ts:t_end;
N = length(time);

% Speed reference profile (step change)
halfN = floor(N/2);
omega_ref = [ones(1,halfN)*100, ones(1,N-halfN)*150]; % rad/s

% Initialize variables
omega = zeros(1,N); % Motor speed
i = zeros(1,N);     % Motor current
u = zeros(1,N);     % Control voltage

%% ========================
%  Simulation Loop
%  ========================
for k = 1:N-1
    best_cost = inf;
    best_u = u(max(k-1,1));  % Start from last applied voltage
    
    for candidate_u = -50:5:50  % Candidate voltages
        omega_pred = omega(k);
        i_pred = i(k);
        cost = 0;
        
        for p = 1:Np
            torque = Kt * i_pred;
            omega_pred = omega_pred + Ts*((torque - B*omega_pred)/J);
            i_pred = i_pred + Ts*((candidate_u - Rs*i_pred)/L);
            
            ref_index = min(k+p, N);
            cost = cost + Q*(omega_ref(ref_index) - omega_pred)^2 + ...
                          R_weight*(candidate_u)^2;
        end
        
        if cost < best_cost
            best_cost = cost;
            best_u = candidate_u;
        end
    end
    
    % Apply best control
    u(k) = best_u;
    
    % Update actual motor dynamics
    torque = Kt * i(k);
    omega(k+1) = omega(k) + Ts*((torque - B*omega(k))/J);
    i(k+1) = i(k) + Ts*((u(k) - Rs*i(k))/L);
end

%% ========================
%  Plot results
%  ========================
figure;

subplot(3,1,1);
plot(time, omega, 'b', 'LineWidth', 1.5); hold on;
plot(time, omega_ref, 'r--', 'LineWidth', 1.5);
ylabel('Speed (rad/s)');
legend('Actual','Reference');
title('SRM Speed Control using Brute-force MPC');
grid on;

subplot(3,1,2);
plot(time, u, 'k', 'LineWidth', 1.5);
ylabel('Voltage (V)');
title('Control Input (Voltage)');
grid on;

subplot(3,1,3);
plot(time, i, 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Current (A)');
title('Motor Current');
grid on;
