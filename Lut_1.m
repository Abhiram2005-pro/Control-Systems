%% SRM Lookup Tables (Flux & Torque)
% Author: [Your Name]
% Date: [Today]

clear; clc;

%% Breakpoints
Bp1_Current = linspace(0,180,181);   % rotor angle [deg]
Bp2_Current = linspace(0,10,101);    % current [A]

%% Initialize LUTs
LuT_Current = zeros(length(Bp1_Current), length(Bp2_Current));
LuT_Torque  = zeros(length(Bp1_Current), length(Bp2_Current));

%% ============================
% 1. Fill Known Experimental Data
% Example data: 22 x 11 block from measurement (replace with your dataset)
% NOTE: This is just the sample you pasted earlier.
data_block = [ ...
0   0.3781   0.7562   1.1343   1.5124   1.8905   2.2686   2.6467   3.0248   3.4030   3.7811
0   0.3791   0.7582   1.1373   1.5164   1.8955   2.2746   2.6537   3.0329   3.4120   3.7911
0   0.3801   0.7602   1.1403   1.5204   1.9005   2.2806   2.6607   3.0409   3.4210   3.8011
0   0.3811   0.7622   1.1433   1.5244   1.9055   2.2866   2.6678   3.0489   3.4300   3.8111
0   0.3821   0.7642   1.1463   1.5284   1.9105   2.2927   2.6748   3.0569   3.4390   3.8211
0   0.3831   0.7662   1.1493   1.5324   1.9155   2.2987   2.6818   3.0649   3.4480   3.8311
0   0.3841   0.7682   1.1523   1.5364   1.9205   2.3047   2.6888   3.0729   3.4570   3.8411
0   0.3851   0.7702   1.1553   1.5404   1.9256   2.3107   2.6958   3.0809   3.4660   3.8511
0   0.3861   0.7722   1.1583   1.5444   1.9306   2.3167   2.7028   3.0889   3.4750   3.8611
0   0.3871   0.7742   1.1613   1.5484   1.9356   2.3227   2.7098   3.0969   3.4840   3.8711
0   0.3881   0.7762   1.1643   1.5525   1.9406   2.3287   2.7168   3.1049   3.4930   3.8811
0   0.3891   0.7782   1.1673   1.5565   1.9456   2.3347   2.7238   3.1129   3.5020   3.8911
0   0.3901   0.7802   1.1703   1.5605   1.9506   2.3407   2.7308   3.1209   3.5110   3.9011
0   0.3911   0.7822   1.1733   1.5645   1.9556   2.3467   2.7378   3.1289   3.5200   3.9111
0   0.3921   0.7842   1.1763   1.5685   1.9606   2.3527   2.7448   3.1369   3.5290   3.9211
0   0.3931   0.7862   1.1794   1.5725   1.9656   2.3587   2.7518   3.1449   3.5380   3.9312
0   0.3941   0.7882   1.1824   1.5765   1.9706   2.3647   2.7588   3.1529   3.5470   3.9412
0   0.3951   0.7902   1.1854   1.5805   1.9756   2.3707   2.7658   3.1609   3.5561   3.9512
0   0.3961   0.7922   1.1884   1.5845   1.9806   2.3767   2.7728   3.1689   3.5651   3.9612
0   0.3971   0.7942   1.1914   1.5885   1.9856   2.3827   2.7798   3.1769   3.5741   3.9712
0   0.3981   0.7962   1.1944   1.5925   1.9906   2.3887   2.7868   3.1849   3.5831   3.9812
0   0.3991   0.7982   1.1974   1.5965   1.9956   2.3947   2.7938   3.1930   3.5921   3.9912
];

% Insert into top-left corner of LUT
LuT_Current(1:size(data_block,1), 1:size(data_block,2)) = data_block;

%% ============================
% 2. Fill Remaining Points by Interpolation/Extrapolation
[AA, CC] = meshgrid(Bp1_Current, Bp2_Current);

% Use last row/col trends to extrapolate
for a = 1:length(Bp1_Current)
    for c = 1:length(Bp2_Current)
        if LuT_Current(a,c) == 0
            % simple extrapolation using linear growth
            LuT_Current(a,c) = LuT_Current(max(1,a-1), max(1,c-1)) + 0.01*(a+c);
        end
    end
end

%% ============================
% 3. Generate Torque LUT (dummy model, replace with measured data)
for a = 1:length(Bp1_Current)
    for c = 1:length(Bp2_Current)
        % Example: torque increases with current and angle sinusoidally
        LuT_Torque(a,c) = 0.1 * Bp2_Current(c) * sin(deg2rad(Bp1_Current(a)));
    end
end

%% ============================
% 4. Save to MAT file for Simulink
save('SRM_LUT.mat','Bp1_Current','Bp2_Current','LuT_Current','LuT_Torque');

disp('✅ Lookup tables generated and saved as SRM_LUT.mat');
