% BUILD_CLEAN_3RIG_MODEL.M
% Constructs Master_Active_Suspension.slx from scratch in MATLAB/Simulink
% Strictly implementing the reference connection model extended to THREE RIGS:
%   Rig 1: Linear Quadratic Regulator (LQR)
%   Rig 2: Fixed Super-Twisting Sliding Mode Controller (Fixed STSMC)
%   Rig 3: Adaptive Super-Twisting Sliding Mode Controller (Adaptive STSMC)

clear variables;
clc;

parameters;

model_name = 'Master_Active_Suspension';
if bdIsLoaded(model_name), close_system(model_name, 0); end
if exist([model_name '.slx'], 'file'), delete([model_name '.slx']); end

new_system(model_name);
load_system(model_name);
set_param(model_name, 'Lock', 'off');

% Solver settings
set_param(model_name, 'InitFcn', 'parameters;');
set_param(model_name, 'PreLoadFcn', 'parameters;');
set_param(model_name, 'StopTime', 'T_stop');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '1e-3');
set_param(model_name, 'ZeroCrossAlgorithm', 'Adaptive');
set_param(model_name, 'AlgebraicLoopMsg', 'none');

%% 1. Road Surface Generator Subsystem
road_sub = [model_name '/Road_Surface_Generator'];
add_block('simulink/Ports & Subsystems/Subsystem', road_sub, 'Position', [40, 260, 160, 360]);
delete_line(road_sub, 'In1/1', 'Out1/1');
delete_block([road_sub '/In1']);
set_param([road_sub '/Out1'], 'Name', 'zr', 'Position', [460, 175, 490, 195]);

% Selector Constant
add_block('simulink/Sources/Constant', [road_sub '/Select_Road'], 'Value', 'ROAD_ID', 'Position', [220, 20, 300, 50]);

% Profile 1: Smooth Undulating Highway (4 mm Sine)
add_block('simulink/Sources/Sine Wave', [road_sub '/Profile 1: Smooth Sine'], ...
    'Amplitude', 'road1_amp', 'Frequency', '2*pi*road1_freq', 'SampleTime', '0', 'Position', [50, 70, 150, 100]);

% Profile 2: Severe Discrete Potholes (-20 mm Pulse Drops)
add_block('simulink/Sources/Pulse Generator', [road_sub '/Profile 2: Pothole Pulse'], ...
    'Amplitude', '-road2_depth', 'Period', '1/road2_freq', 'PulseWidth', 'road2_width', 'PhaseDelay', '2.0', ...
    'Position', [50, 130, 150, 160]);

% Profile 3: Dynamic Frequency-Sweep Chirp (0.1 - 10 Hz)
add_block('simulink/Sources/Chirp Signal', [road_sub '/Profile 3: Chirp'], ...
    'f1', 'road3_f0', 'T', 'T_stop', 'f2', 'road3_f1', 'Position', [50, 190, 150, 220]);
add_block('simulink/Math Operations/Gain', [road_sub '/Gain_Chirp_Amp'], 'Gain', 'road3_amp', 'Position', [175, 195, 215, 215]);
add_line(road_sub, 'Profile 3: Chirp/1', 'Gain_Chirp_Amp/1');

% Profile 4: Stochastic ISO Class D (4.5 Hz broadband)
add_block('simulink/Sources/Sine Wave', [road_sub '/Profile 4: ISO Class D'], ...
    'Amplitude', 'road4_amp', 'Frequency', '2*pi*4.5', 'SampleTime', '0', 'Position', [50, 250, 150, 280]);

% Profile 5: Harmonic Resonance Corrugation (2.58 Hz)
add_block('simulink/Sources/Sine Wave', [road_sub '/Profile 5: Resonance (2.58 Hz)'], ...
    'Amplitude', 'road5_amp', 'Frequency', '2*pi*road5_freq', 'SampleTime', '0', 'Position', [50, 310, 150, 340]);

% Multiport Switch
add_block('simulink/Signal Routing/Multiport Switch', [road_sub '/Mux_Switch'], ...
    'Inputs', '5', 'DataPortOrder', 'One-based contiguous', 'Position', [350, 30, 390, 360]);

add_line(road_sub, 'Select_Road/1', 'Mux_Switch/1');
add_line(road_sub, 'Profile 1: Smooth Sine/1', 'Mux_Switch/2');
add_line(road_sub, 'Profile 2: Pothole Pulse/1', 'Mux_Switch/3');
add_line(road_sub, 'Gain_Chirp_Amp/1', 'Mux_Switch/4');
add_line(road_sub, 'Profile 4: ISO Class D/1', 'Mux_Switch/5');
add_line(road_sub, 'Profile 5: Resonance (2.58 Hz)/1', 'Mux_Switch/6');
add_line(road_sub, 'Mux_Switch/1', 'zr/1');

%% 2. Differentiator (zr -> zr_dot)
load_system('Master_Active_Suspension_Simulation');
add_block('Master_Active_Suspension_Simulation/Differentiator', [model_name '/Differentiator'], 'Position', [200, 290, 260, 330]);
close_system('Master_Active_Suspension_Simulation', 0);
add_line(model_name, 'Road_Surface_Generator/1', 'Differentiator/1');

%% 3. Three Parallel Rigs (LQR, Fixed SMC, Adaptive SMC)
rigs = { ...
    'LQR',          'Rig_LQR',          80;  ...
    'Fixed_SMC',    'Rig_Fixed_SMC',    280; ...
    'Adaptive_SMC', 'Rig_Adaptive_SMC', 480  ...
};

for r = 1:size(rigs, 1)
    tag = rigs{r, 1};
    name = rigs{r, 2};
    y0 = rigs{r, 3};
    
    % Input Mux: [zr_dot; u_force]
    mux_blk = [model_name '/Mux_' tag];
    add_block('simulink/Signal Routing/Mux', mux_blk, 'Inputs', '2', 'Position', [340, y0+30, 345, y0+70]);
    
    % Connect road velocity to Mux port 1
    add_line(model_name, 'Differentiator/1', ['Mux_' tag '/1'], 'autorouting', 'on');
    
    % State-Space Plant
    ss_blk = [model_name '/Plant_' tag];
    add_block('simulink/Continuous/State-Space', ss_blk, ...
        'A', 'A', 'B', 'B', 'C', 'C_sim', 'D', 'D_sim', 'X0', 'zeros(4,1)', ...
        'Position', [380, y0+15, 480, y0+85]);
    add_line(model_name, ['Mux_' tag '/1'], ['Plant_' tag '/1']);
    
    % Demux to split plant outputs:
    % Port 1: Full state x (4x1)
    % Port 2: Body Acceleration zs_ddot (1x1)
    % Port 3: Suspension Deflection defl (1x1)
    % Port 4: Dynamic Tire Deflection tire (1x1)
    demux_blk = [model_name '/Demux_' tag];
    add_block('simulink/Signal Routing/Demux', demux_blk, 'Outputs', '[4, 1, 1, 1]', 'Position', [510, y0+10, 515, y0+90]);
    add_line(model_name, ['Plant_' tag '/1'], ['Demux_' tag '/1']);
    
    % Saturation Block on Actuator Force (+/- 39.2 N)
    sat_blk = [model_name '/Sat_' tag];
    add_block('simulink/Discontinuities/Saturation', sat_blk, ...
        'UpperLimit', 'F_max', 'LowerLimit', '-F_max', 'Position', [270, y0+110, 300, y0+130]);
    
    % Loop saturated force back into Mux port 2
    add_line(model_name, ['Sat_' tag '/1'], ['Mux_' tag '/2'], 'autorouting', 'on');
    
    % Specific Controller Implementation
    switch tag
        case 'LQR'
            % u = -K_lqr * x
            gain_lqr = [model_name '/Gain_K_LQR'];
            add_block('simulink/Math Operations/Gain', gain_lqr, ...
                'Gain', '-K_lqr', 'Multiplication', 'Matrix(K*u)', 'Position', [360, y0+105, 440, y0+135]);
            add_line(model_name, ['Demux_' tag '/1'], 'Gain_K_LQR/1', 'autorouting', 'on');
            add_line(model_name, 'Gain_K_LQR/1', ['Sat_' tag '/1'], 'autorouting', 'on');
            
        case 'Fixed_SMC'
            % Super-Twisting Subsystem (s = x2 + c1*x1, u = -k1*|s|^0.5*tanh(s/eps) + v)
            fsmc_sub = [model_name '/Controller_Fixed_SMC'];
            add_block('simulink/Ports & Subsystems/Subsystem', fsmc_sub, 'Position', [340, y0+100, 460, y0+140]);
            delete_line(fsmc_sub, 'In1/1', 'Out1/1');
            set_param([fsmc_sub '/In1'], 'Name', 'x_state', 'Position', [30, 80, 60, 100]);
            set_param([fsmc_sub '/Out1'], 'Name', 'u_force', 'Position', [650, 80, 680, 100]);
            
            add_block('simulink/Signal Routing/Demux', [fsmc_sub '/Demux4'], 'Outputs', '4', 'Position', [90, 60, 95, 120]);
            add_line(fsmc_sub, 'x_state/1', 'Demux4/1');
            
            % s = x2 + c1*x1
            add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_c1'], 'Gain', 'c1_fixed', 'Position', [130, 50, 170, 80]);
            add_block('simulink/Math Operations/Sum', [fsmc_sub '/Sum_s'], 'Inputs', '++', 'Position', [200, 65, 220, 95]);
            add_line(fsmc_sub, 'Demux4/1', 'Gain_c1/1');
            add_line(fsmc_sub, 'Gain_c1/1', 'Sum_s/1');
            add_line(fsmc_sub, 'Demux4/2', 'Sum_s/2');
            
            % tanh(s / eps)
            add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_inv_eps'], 'Gain', '1/eps_smc', 'Position', [250, 70, 290, 100]);
            add_block('simulink/Math Operations/Trigonometric Function', [fsmc_sub '/Tanh_s'], 'Operator', 'tanh', 'Position', [315, 70, 345, 100]);
            add_line(fsmc_sub, 'Sum_s/1', 'Gain_inv_eps/1');
            add_line(fsmc_sub, 'Gain_inv_eps/1', 'Tanh_s/1');
            
            % sqrt(|s|)
            add_block('simulink/Math Operations/Abs', [fsmc_sub '/Abs_s'], 'ZeroCross', 'off', 'Position', [250, 20, 280, 50]);
            add_block('simulink/Math Operations/Math Function', [fsmc_sub '/Sqrt_s'], 'Function', 'sqrt', 'Position', [315, 20, 345, 50]);
            add_line(fsmc_sub, 'Sum_s/1', 'Abs_s/1');
            add_line(fsmc_sub, 'Abs_s/1', 'Sqrt_s/1');
            
            % Branch 1: -k1 * sqrt(|s|) * tanh(s/eps)
            add_block('simulink/Math Operations/Product', [fsmc_sub '/Prod_prop'], 'Inputs', '**', 'Position', [380, 25, 405, 60]);
            add_line(fsmc_sub, 'Sqrt_s/1', 'Prod_prop/1');
            add_line(fsmc_sub, 'Tanh_s/1', 'Prod_prop/2');
            add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_k1'], 'Gain', '-k1_fixed', 'Position', [435, 30, 480, 60]);
            add_line(fsmc_sub, 'Prod_prop/1', 'Gain_k1/1');
            
            % Branch 2: Integral branch v = int(-k2 * tanh(s/eps))
            add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_k2'], 'Gain', '-k2_fixed', 'Position', [380, 110, 425, 140]);
            add_block('simulink/Continuous/Integrator', [fsmc_sub '/Integrator_v'], ...
                'LimitOutput', 'on', 'UpperSaturationLimit', 'F_max', 'LowerSaturationLimit', '-F_max', ...
                'Position', [450, 110, 480, 140]);
            add_line(fsmc_sub, 'Tanh_s/1', 'Gain_k2/1');
            add_line(fsmc_sub, 'Gain_k2/1', 'Integrator_v/1');
            
            % Sum branches: u = branch1 + branch2
            add_block('simulink/Math Operations/Sum', [fsmc_sub '/Sum_u'], 'Inputs', '++', 'Position', [530, 65, 555, 95]);
            add_line(fsmc_sub, 'Gain_k1/1', 'Sum_u/1');
            add_line(fsmc_sub, 'Integrator_v/1', 'Sum_u/2');
            add_line(fsmc_sub, 'Sum_u/1', 'u_force/1');
            
            add_line(model_name, ['Demux_' tag '/1'], 'Controller_Fixed_SMC/1', 'autorouting', 'on');
            add_line(model_name, 'Controller_Fixed_SMC/1', ['Sat_' tag '/1'], 'autorouting', 'on');
            
        case 'Adaptive_SMC'
            % Online Adaptive Super-Twisting Subsystem
            asmc_sub = [model_name '/Controller_Adaptive_SMC'];
            add_block('simulink/Ports & Subsystems/Subsystem', asmc_sub, 'Position', [340, y0+100, 460, y0+140]);
            delete_line(asmc_sub, 'In1/1', 'Out1/1');
            set_param([asmc_sub '/In1'], 'Name', 'x_state', 'Position', [30, 80, 60, 100]);
            set_param([asmc_sub '/Out1'], 'Name', 'u_force', 'Position', [650, 80, 680, 100]);
            
            add_block('simulink/Signal Routing/Demux', [asmc_sub '/Demux4'], 'Outputs', '4', 'Position', [90, 60, 95, 120]);
            add_line(asmc_sub, 'x_state/1', 'Demux4/1');
            
            % s = x2 + c1_adap*x1
            add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_c1'], 'Gain', 'c1_adap', 'Position', [130, 50, 170, 80]);
            add_block('simulink/Math Operations/Sum', [asmc_sub '/Sum_s'], 'Inputs', '++', 'Position', [200, 65, 220, 95]);
            add_line(asmc_sub, 'Demux4/1', 'Gain_c1/1');
            add_line(asmc_sub, 'Gain_c1/1', 'Sum_s/1');
            add_line(asmc_sub, 'Demux4/2', 'Sum_s/2');
            
            % tanh(s / eps)
            add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_inv_eps'], 'Gain', '1/eps_smc', 'Position', [250, 70, 290, 100]);
            add_block('simulink/Math Operations/Trigonometric Function', [asmc_sub '/Tanh_s'], 'Operator', 'tanh', 'Position', [315, 70, 345, 100]);
            add_line(asmc_sub, 'Sum_s/1', 'Gain_inv_eps/1');
            add_line(asmc_sub, 'Gain_inv_eps/1', 'Tanh_s/1');
            
            % sqrt(|s|)
            add_block('simulink/Math Operations/Abs', [asmc_sub '/Abs_s'], 'ZeroCross', 'off', 'Position', [250, 20, 280, 50]);
            add_block('simulink/Math Operations/Math Function', [asmc_sub '/Sqrt_s'], 'Function', 'sqrt', 'Position', [315, 20, 345, 50]);
            add_line(asmc_sub, 'Sum_s/1', 'Abs_s/1');
            add_line(asmc_sub, 'Abs_s/1', 'Sqrt_s/1');
            
            % Online Gain Adaptation for k1_hat:
            % dot_k1 = gamma * sqrt(|s|) - sigma * (k1 - k1_min)
            add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_gamma'], 'Gain', 'gamma_adap', 'Position', [380, 160, 420, 190]);
            add_line(asmc_sub, 'Sqrt_s/1', 'Gain_gamma/1', 'autorouting', 'on');
            
            add_block('simulink/Continuous/Integrator', [asmc_sub '/Integrator_k1'], ...
                'InitialCondition', 'k1_init', 'LimitOutput', 'on', 'UpperSaturationLimit', 'k1_max', 'LowerSaturationLimit', 'k1_min', ...
                'Position', [450, 160, 480, 190]);
            add_line(asmc_sub, 'Gain_gamma/1', 'Integrator_k1/1');
            
            % k2_hat = k2_ratio * k1_hat
            add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_k2_ratio'], 'Gain', 'k2_ratio', 'Position', [450, 210, 490, 240]);
            add_line(asmc_sub, 'Integrator_k1/1', 'Gain_k2_ratio/1');
            
            % Branch 1: -k1_hat * sqrt(|s|) * tanh(s/eps)
            add_block('simulink/Math Operations/Product', [asmc_sub '/Prod_prop'], 'Inputs', '***', 'Position', [480, 25, 505, 75]);
            add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_neg1'], 'Gain', '-1', 'Position', [520, 40, 540, 60]);
            add_line(asmc_sub, 'Sqrt_s/1', 'Prod_prop/1');
            add_line(asmc_sub, 'Tanh_s/1', 'Prod_prop/2');
            add_line(asmc_sub, 'Integrator_k1/1', 'Prod_prop/3', 'autorouting', 'on');
            add_line(asmc_sub, 'Prod_prop/1', 'Gain_neg1/1');
            
            % Branch 2: Integral branch v = int(-k2_hat * tanh(s/eps))
            add_block('simulink/Math Operations/Product', [asmc_sub '/Prod_k2'], 'Inputs', '**', 'Position', [520, 110, 545, 140]);
            add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_neg2'], 'Gain', '-1', 'Position', [560, 115, 580, 135]);
            add_line(asmc_sub, 'Gain_k2_ratio/1', 'Prod_k2/1', 'autorouting', 'on');
            add_line(asmc_sub, 'Tanh_s/1', 'Prod_k2/2', 'autorouting', 'on');
            add_line(asmc_sub, 'Prod_k2/1', 'Gain_neg2/1');
            
            add_block('simulink/Continuous/Integrator', [asmc_sub '/Integrator_v'], ...
                'LimitOutput', 'on', 'UpperSaturationLimit', 'F_max', 'LowerSaturationLimit', '-F_max', ...
                'Position', [600, 110, 630, 140]);
            add_line(asmc_sub, 'Gain_neg2/1', 'Integrator_v/1');
            
            % Sum branches: u = branch1 + branch2
            add_block('simulink/Math Operations/Sum', [asmc_sub '/Sum_u'], 'Inputs', '++', 'Position', [650, 45, 675, 75]);
            add_line(asmc_sub, 'Gain_neg1/1', 'Sum_u/1');
            add_line(asmc_sub, 'Integrator_v/1', 'Sum_u/2', 'autorouting', 'on');
            add_line(asmc_sub, 'Sum_u/1', 'u_force/1');
            
            add_line(model_name, ['Demux_' tag '/1'], 'Controller_Adaptive_SMC/1', 'autorouting', 'on');
            add_line(model_name, 'Controller_Adaptive_SMC/1', ['Sat_' tag '/1'], 'autorouting', 'on');
    end
end

%% 4. Multi-Trace Comparison Scopes
scopes = { ...
    'Scope_BodyAccel', 3, [700, 120, 740, 160]; ...
    'Scope_SuspDefl',  3, [700, 220, 740, 260]; ...
    'Scope_Force',     3, [700, 320, 740, 360]  ...
};
for s = 1:size(scopes, 1)
    sc = [model_name '/' scopes{s, 1}];
    add_block('simulink/Sinks/Scope', sc, 'NumInputPorts', num2str(scopes{s, 2}), 'Position', scopes{s, 3});
end

add_line(model_name, 'Demux_LQR/2',          'Scope_BodyAccel/1', 'autorouting', 'on');
add_line(model_name, 'Demux_Fixed_SMC/2',    'Scope_BodyAccel/2', 'autorouting', 'on');
add_line(model_name, 'Demux_Adaptive_SMC/2', 'Scope_BodyAccel/3', 'autorouting', 'on');

add_line(model_name, 'Demux_LQR/3',          'Scope_SuspDefl/1',  'autorouting', 'on');
add_line(model_name, 'Demux_Fixed_SMC/3',    'Scope_SuspDefl/2',  'autorouting', 'on');
add_line(model_name, 'Demux_Adaptive_SMC/3', 'Scope_SuspDefl/3',  'autorouting', 'on');

add_line(model_name, 'Sat_LQR/1',            'Scope_Force/1',     'autorouting', 'on');
add_line(model_name, 'Sat_Fixed_SMC/1',      'Scope_Force/2',     'autorouting', 'on');
add_line(model_name, 'Sat_Adaptive_SMC/1',   'Scope_Force/3',     'autorouting', 'on');

%% 5. To-Workspace Data Loggers
loggers = { ...
    'Road_Surface_Generator/1', 'zr_sim',         [220, 380, 280, 400]; ...
    'Demux_LQR/2',              'zs_ddot_lqr',    [580, 70,  660, 90];  ...
    'Demux_Fixed_SMC/2',        'zs_ddot_fsmc',   [580, 270, 660, 290]; ...
    'Demux_Adaptive_SMC/2',     'zs_ddot_asmc',   [580, 470, 660, 490]; ...
    'Demux_LQR/3',              'defl_lqr',       [580, 95,  660, 115]; ...
    'Demux_Fixed_SMC/3',        'defl_fsmc',      [580, 295, 660, 315]; ...
    'Demux_Adaptive_SMC/3',     'defl_asmc',      [580, 495, 660, 515]; ...
    'Demux_LQR/4',              'tire_lqr',       [580, 120, 660, 140]; ...
    'Demux_Fixed_SMC/4',        'tire_fsmc',      [580, 320, 660, 340]; ...
    'Demux_Adaptive_SMC/4',     'tire_asmc',      [580, 520, 660, 540]; ...
    'Sat_LQR/1',                'u_lqr',          [180, 145, 240, 165]; ...
    'Sat_Fixed_SMC/1',          'u_fsmc',         [180, 345, 240, 365]; ...
    'Sat_Adaptive_SMC/1',       'u_asmc',         [180, 545, 240, 565]  ...
};

for k = 1:size(loggers, 1)
    src_p = loggers{k, 1};
    vname = loggers{k, 2};
    pos_v = loggers{k, 3};
    lblk  = [model_name '/' vname];
    add_block('simulink/Sinks/To Workspace', lblk, ...
        'VariableName', vname, 'SaveFormat', 'Array', 'Position', pos_v);
    add_line(model_name, src_p, [vname '/1'], 'autorouting', 'on');
end

save_system(model_name, fullfile(pwd, [model_name '.slx']));
close_system(model_name, 0);

fprintf('Clean 3-Rig Master_Active_Suspension.slx built successfully!\n');
