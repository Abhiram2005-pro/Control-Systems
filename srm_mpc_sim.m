%% srm_mpc_sim.m
% Run AFTER srm_mpc_setup.m
clc;

assert(evalin('base','exist(''mpcobj'',''var'')==1'), 'Run srm_mpc_setup.m first');
mpcobj = evalin('base','mpcobj');
sysd   = evalin('base','sysd');
Ts     = evalin('base','Ts');

[A,B,C,D] = ssdata(sysd);   % A:2x2, B:2x2, C:1x2, D:1x2  (y=scalar speed)

% Sim horizon
Tend = 1.5;                     % seconds
N    = round(Tend/Ts);
t    = (0:N-1)'*Ts;

% References and measured disturbance sequence
r   = 200*ones(N,1);            % 200 rad/s
r(t>0.8) = 250;                 % step up at 0.8 s
TL  = zeros(N,1);               % MD (Nm)
TL(t>0.5) = 0.5;                % apply 0.5 Nm load at 0.5 s

% Storage
x    = zeros(2,1);              % plant state [i; w]
xm   = mpcstate(mpcobj);        % MPC internal state (Kalman etc.)
ylog = zeros(N,1);
ulog = zeros(N,1);
ilog = zeros(N,1);
wlog = zeros(N,1);

for k = 1:N
    y = C*x;                        % measured speed
    v = TL(k);                      % measured disturbance (MD)
    % Compute optimal MV
    u = mpcmove(mpcobj, xm, y, r(k), v);
    % Plant update: x+ = A*x + B*[u; v]
    x = A*x + B*[u; v];
    % Log
    ylog(k) = y;
    ulog(k) = u;
    ilog(k) = x(1);
    wlog(k) = x(2);
end

% Plots
figure('Color','w');
subplot(3,1,1); plot(t,wlog,'b',t,r,'r--'); grid on;
ylabel('\omega (rad/s)'); legend('speed','ref'); title('SRM speed with MPC');
subplot(3,1,2); plot(t,ulog,'k'); grid on; ylabel('V (Volt)'); title('Control (MV)');
subplot(3,1,3); plot(t,ilog,'m'); grid on; ylabel('i (A)'); xlabel('Time (s)'); title('Phase current');
