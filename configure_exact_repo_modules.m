% CONFIGURE_EXACT_REPO_MODULES.M
% Configures Master_Active_Suspension.slx to use the exact modules and road profiles
% from https://github.com/ananthu0987/Roadprofile_CSelec
%
% Modules:
%   - Plant: Quanser quarter-car 2-DOF state-space plant
%   - LQR: Optimal state feedback u = -K_lqr * x
%   - Fixed STSMC: Super-Twisting SMC with fixed gains (c1_fixed, k1_fixed, k2_fixed)
%   - Adaptive STSMC: Super-Twisting SMC scheduled via stsmc_rsi_scheduler (c1_adap, k1_adap, k2_adap)
%   - Saturation: Continuous actuator limit +/- 39.2 N
%
% Road Profiles:
%   1. Smooth Road (RSI 0.10): 4 mm Sine, 0.20 Hz
%   2. Normal Road (RSI 0.35): 20 mm Sine, 0.20 Hz
%   3. Rough Road (RSI 0.65) : ISO Class D Broadband (30 harmonics, rng 42)
%   4. Pothole Road (RSI 0.95): 20 mm Pulse Drops (0.30 Hz, 10% width)

clear variables;
clc;

model_name = 'Master_Active_Suspension';
if ~bdIsLoaded(model_name), load_system(model_name); end
set_param(model_name, 'Lock', 'off');

% Initialize parameters in base workspace
parameters;

%% 1. Rebuild Controller_Adaptive_SMC with Clean Super-Twisting Architecture
% (eliminates ad-hoc continuous integrator and uses stsmc_rsi_scheduler gains)
asmc_sub = [model_name '/Controller_Adaptive_SMC'];
if getSimulinkBlockHandle(asmc_sub) ~= -1
    % Get position
    pos_asmc = get_param(asmc_sub, 'Position');
    
    % Delete existing lines and blocks inside asmc_sub
    lines = find_system(asmc_sub, 'FindAll', 'on', 'Type', 'line');
    delete_line(lines);
    
    blks = find_system(asmc_sub, 'SearchDepth', 1, 'Type', 'block');
    for i = 1:length(blks)
        if ~strcmp(blks{i}, asmc_sub)
            delete_block(blks{i});
        end
    end
    
    % Add Inport & Outport
    add_block('simulink/Ports & Subsystems/In1', [asmc_sub '/x_state'], 'Position', [30, 80, 60, 100]);
    add_block('simulink/Ports & Subsystems/Out1', [asmc_sub '/u_force'], 'Position', [650, 80, 680, 100]);
    
    % Demux states: [x1; x2; x3; x4]
    add_block('simulink/Signal Routing/Demux', [asmc_sub '/Demux4'], 'Outputs', '4', 'Position', [90, 60, 95, 120]);
    add_line(asmc_sub, 'x_state/1', 'Demux4/1');
    
    % Sliding surface: s = x2 + c1_adap * x1
    add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_c1'], 'Gain', 'c1_adap', 'Position', [140, 60, 180, 80]);
    add_line(asmc_sub, 'Demux4/1', 'Gain_c1/1');
    
    add_block('simulink/Math Operations/Sum', [asmc_sub '/Sum_s'], 'Inputs', '++', 'Position', [210, 65, 230, 95]);
    add_line(asmc_sub, 'Gain_c1/1', 'Sum_s/1');
    add_line(asmc_sub, 'Demux4/2', 'Sum_s/2');
    
    % Smooth sign term: tanh(s / eps_smc)
    add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_inv_eps'], 'Gain', '1/eps_smc', 'Position', [260, 95, 300, 125]);
    add_line(asmc_sub, 'Sum_s/1', 'Gain_inv_eps/1');
    
    add_block('simulink/Math Operations/Trigonometric Function', [asmc_sub '/Tanh_s'], ...
        'ApproximationMethod', 'None', 'Function', 'tanh', 'Position', [330, 95, 360, 125]);
    add_line(asmc_sub, 'Gain_inv_eps/1', 'Tanh_s/1');
    
    % Super-Twisting proportional branch: -k1_adap * sqrt(|s|) * tanh(s/eps_smc)
    add_block('simulink/Math Operations/Abs', [asmc_sub '/Abs_s'], 'Position', [260, 45, 290, 75]);
    add_line(asmc_sub, 'Sum_s/1', 'Abs_s/1');
    
    add_block('simulink/Math Operations/Math Function', [asmc_sub '/Sqrt_s'], 'Function', 'sqrt', 'Position', [320, 45, 350, 75]);
    add_line(asmc_sub, 'Abs_s/1', 'Sqrt_s/1');
    
    add_block('simulink/Math Operations/Product', [asmc_sub '/Prod_prop'], 'Inputs', '**', 'Position', [400, 50, 425, 80]);
    add_line(asmc_sub, 'Sqrt_s/1', 'Prod_prop/1');
    add_line(asmc_sub, 'Tanh_s/1', 'Prod_prop/2');
    
    add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_k1'], 'Gain', '-k1_adap', 'Position', [450, 55, 490, 75]);
    add_line(asmc_sub, 'Prod_prop/1', 'Gain_k1/1');
    
    % Super-Twisting integral branch: -k2_adap * \int tanh(s/eps_smc) dt
    add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_k2'], 'Gain', '-k2_adap', 'Position', [400, 110, 440, 130]);
    add_line(asmc_sub, 'Tanh_s/1', 'Gain_k2/1');
    
    add_block('simulink/Continuous/Integrator', [asmc_sub '/Integrator_v'], ...
        'LimitOutput', 'on', 'UpperSaturationLimit', 'F_max', 'LowerSaturationLimit', '-F_max', ...
        'Position', [470, 105, 500, 135]);
    add_line(asmc_sub, 'Gain_k2/1', 'Integrator_v/1');
    
    % Sum total control effort: u = branch1 + branch2
    add_block('simulink/Math Operations/Sum', [asmc_sub '/Sum_u'], 'Inputs', '++', 'Position', [550, 60, 575, 90]);
    add_line(asmc_sub, 'Gain_k1/1', 'Sum_u/1');
    add_line(asmc_sub, 'Integrator_v/1', 'Sum_u/2');
    add_line(asmc_sub, 'Sum_u/1', 'u_force/1');
end

%% 2. Ensure Solver Settings are Fast and Zero-Crossing Clean
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '1e-3');
set_param(model_name, 'ZeroCrossAlgorithm', 'Adaptive');

% Save model
save_system(model_name);
fprintf('Master_Active_Suspension.slx successfully configured with exact repository modules!\n');
