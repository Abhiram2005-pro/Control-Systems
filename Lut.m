clc;
clear;
close all;

%% ========================
%  Motor Parameter (example)
%  ========================
J = 0.01;   % Inertia (kg·m^2)

%% ========================
%  Breakpoints
%  ========================
Bp1_Current = linspace(0,180,181);   % Angle (°) → 181 points
Bp2_Current = linspace(0,0.50,101);  % Current (A) → 101 points

%% ========================
%  Shape Controls
%  ========================
peakGain   = 820;   % plateau height (~800 in your plots)
angleDecay = 55;    % angle decay factor
satKnee    = 0.12;  % knee current where saturation starts

%% ========================
%  Build shaping grid (0..100° × 0..0.5A, for smoothness)
%  ========================
Bp1_shape = linspace(0,100,101);
[ThShape, IShape] = ndgrid(Bp1_shape, Bp2_Current);

% Envelope vs angle: rise + shoulder + decay
envAngle = exp(-(ThShape/angleDecay).^1.3) + ...
           0.35*exp(-((ThShape-20)/12).^2);

% Saturation vs current
satCurr = 1 - exp(-(IShape/satKnee));

% Mild surface modulation
shapeMod = 0.90 + 0.12*cos(pi*ThShape/40);

% Base surface
LuT_Current_shape = peakGain * envAngle .* satCurr .* shapeMod;
LuT_Current_shape(LuT_Current_shape<0) = 0;

%% ========================
%  Resample onto 181 × 101 grid (angle × current)
%  ========================
LuT_Current = zeros(length(Bp1_Current), length(Bp2_Current));
for col = 1:length(Bp2_Current)
    LuT_Current(:,col) = interp1(Bp1_shape, LuT_Current_shape(:,col), ...
                                 Bp1_Current, 'pchip');
end

%% ========================
%  Torque Lookup (synthetic example)
%  ========================
torqueScale = 3.5;  % Nm scaling
LuT_Torque_shape = torqueScale * (satCurr.^1.6) .* envAngle;

LuT_Torque = zeros(length(Bp1_Current), length(Bp2_Current));
for col = 1:length(Bp2_Current)
    LuT_Torque(:,col) = interp1(Bp1_shape, LuT_Torque_shape(:,col), ...
                                Bp1_Current, 'pchip');
end

%% ========================
%  Plot for Verification
%  ========================
[Th, I] = ndgrid(Bp1_Current, Bp2_Current);

figure('Color','w');
subplot(1,2,1);
mesh(Th, I, LuT_Current); grid on;
xlabel('Angle (deg)'); ylabel('Current (A)'); zlabel('LuT\_Current');
title('Lookup Surface: Current');

subplot(1,2,2);
mesh(Th, I, LuT_Torque); grid on;
xlabel('Angle (deg)'); ylabel('Current (A)'); zlabel('LuT\_Torque');
title('Lookup Surface: Torque');

%% ========================
%  Save for Simulink
%  ========================
save('lut_surfaces.mat','Bp1_Current','Bp2_Current','LuT_Current','LuT_Torque');
disp('✅ Lookup tables saved to lut_surfaces.mat');
disp("LuT_Current size: " + mat2str(size(LuT_Current)));
disp("LuT_Torque size: " + mat2str(size(LuT_Torque)));
