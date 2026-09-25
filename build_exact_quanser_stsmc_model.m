% BUILD_EXACT_QUANSER_STSMC_MODEL
% Restores and builds the EXACT Quanser Active Suspension Architecture with Pure STSMC Controller
%
% Strictly STSMC - All LQR artifacts removed.
% Preserves:
%   - Exact original "Road Surface Generator" subsystem (Smooth, Normal, Rough, Pothole)
%   - Exact original "Differentiator" (2nd-Order Low-Pass Filter)
%   - Exact original "Open-loop Simulation" State-Space plant
%   - Exact original "Scopes" subsystem & all 4 Quanser scopes:
%       * "Simulation zr_zs_zus" (Scope)
%       * "Suspension deflection" (Scope)
%       * "Acceleration_Simulation zs_ddot" (Scope)
%       * "Fc_sim" (Scope)
%   - Exact original Saturation (+/- 39.2 N) & Actuator dynamics
%
% Implements:
%   - Pure Native Simulink Super-Twisting Sliding Mode Controller (STSMC)
%       Sliding Surface: s = c1 * (zs - zus) + zs_dot = c1 * x1 + x2
%       Control Law:    u = -k1 * sqrt(|s|) * tanh(s/eps) - k2 * \int tanh(s/eps) dt

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  BUILDING EXACT QUANSER ACTIVE SUSPENSION PURE STSMC ARCHITECTURE\n');
fprintf('========================================================================\n\n');

orig_model = 'Master_Active_Suspension_Simulation_Original';
model_name = 'Master_Active_Suspension_Simulation';

% Ensure models are closed before building
if bdIsLoaded(orig_model)
    close_system(orig_model, 0);
end
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Initialize workspace parameters
setup_as;

% Load original template
load_system(orig_model);

% Clone exact original file
save_system(orig_model, fullfile(pwd, [model_name '.slx']));
close_system(orig_model, 0);

% Open target cloned model
load_system(model_name);
set_param(model_name, 'Lock', 'off');

% Set model callbacks
set_param(model_name, 'InitFcn', 'setup_as;');
set_param(model_name, 'PreLoadFcn', 'setup_as;');
set_param(model_name, 'StopTime', '10.0');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '1e-3');

fprintf('[1/3] Base Quanser model loaded and preserved.\n');

% 1. Cleanly remove old legacy blocks (Manual Switch, Gain5, Constant)
sat_blk = [model_name '/Saturation'];
if getSimulinkBlockHandle(sat_blk) ~= -1
    sat_ph = get_param(sat_blk, 'PortHandles');
    sat_in_line = get_param(sat_ph.Inport(1), 'Line');
    if sat_in_line ~= -1
        delete_line(sat_in_line);
    end
end

if getSimulinkBlockHandle([model_name '/Manual Switch']) ~= -1
    ms_ph = get_param([model_name '/Manual Switch'], 'PortHandles');
    for p = [ms_ph.Inport, ms_ph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, delete_line(l); end
    end
    delete_block([model_name '/Manual Switch']);
end

if getSimulinkBlockHandle([model_name '/Gain5']) ~= -1
    g5_ph = get_param([model_name '/Gain5'], 'PortHandles');
    for p = [g5_ph.Inport, g5_ph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, delete_line(l); end
    end
    delete_block([model_name '/Gain5']);
end

if getSimulinkBlockHandle([model_name '/Constant']) ~= -1
    c_ph = get_param([model_name '/Constant'], 'PortHandles');
    for p = [c_ph.Inport, c_ph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, delete_line(l); end
    end
    delete_block([model_name '/Constant']);
end

fprintf('[2/3] Adding Pure Native STSMC Controller Subsystem...\n');

% 2. Build Pure Native STSMC Subsystem
stsmc_sub = [model_name '/STSMC_Controller'];
if getSimulinkBlockHandle(stsmc_sub) ~= -1
    delete_block(stsmc_sub);
end

add_block('simulink/Ports & Subsystems/Subsystem', stsmc_sub, ...
    'Position', [140, 150, 220, 200]);

delete_line(stsmc_sub, 'In1/1', 'Out1/1');
set_param([stsmc_sub '/In1'], 'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([stsmc_sub '/Out1'], 'Name', 'u_stsmc', 'Position', [620, 100, 650, 120]);

% Demux 4 states: [x1; x2; x3; x4]
add_block('simulink/Signal Routing/Demux', [stsmc_sub '/Demux_states'], ...
    'Outputs', '4', 'Position', [90, 80, 95, 140]);
add_line(stsmc_sub, 'x_state/1', 'Demux_states/1');

% Sliding Surface: s = c1 * x1 + x2
add_block('simulink/Math Operations/Gain', [stsmc_sub '/Gain_c1'], ...
    'Gain', 'c1', 'Position', [130, 75, 170, 105]);
add_block('simulink/Math Operations/Sum', [stsmc_sub '/Sum_s'], ...
    'Inputs', '++', 'Position', [200, 90, 220, 120]);

add_line(stsmc_sub, 'Demux_states/1', 'Gain_c1/1');
add_line(stsmc_sub, 'Gain_c1/1', 'Sum_s/1');
add_line(stsmc_sub, 'Demux_states/2', 'Sum_s/2');

% Smooth Continuous Sign Approximation: tanh(s / eps_bl)
add_block('simulink/Math Operations/Gain', [stsmc_sub '/Gain_inv_eps'], ...
    'Gain', '1/eps_bl', 'Position', [250, 95, 290, 125]);
add_block('simulink/Math Operations/Trigonometric Function', [stsmc_sub '/Tanh_sign_s'], ...
    'Operator', 'tanh', 'Position', [315, 95, 345, 125]);

add_line(stsmc_sub, 'Sum_s/1', 'Gain_inv_eps/1');
add_line(stsmc_sub, 'Gain_inv_eps/1', 'Tanh_sign_s/1');

% Super-Twisting Proportional Term: -k1 * sqrt(|s|) * tanh(s/eps_bl)
add_block('simulink/Math Operations/Abs', [stsmc_sub '/Abs_s'], ...
    'Position', [250, 40, 280, 70]);
add_block('simulink/Math Operations/Math Function', [stsmc_sub '/Sqrt_abs_s'], ...
    'Function', 'sqrt', 'Position', [315, 40, 345, 70]);

add_line(stsmc_sub, 'Sum_s/1', 'Abs_s/1');
add_line(stsmc_sub, 'Abs_s/1', 'Sqrt_abs_s/1');

add_block('simulink/Math Operations/Product', [stsmc_sub '/Prod_prop'], ...
    'Inputs', '**', 'Position', [380, 45, 405, 80]);
add_line(stsmc_sub, 'Sqrt_abs_s/1', 'Prod_prop/1');
add_line(stsmc_sub, 'Tanh_sign_s/1', 'Prod_prop/2');

add_block('simulink/Math Operations/Gain', [stsmc_sub '/Gain_k1'], ...
    'Gain', '-k1', 'Position', [435, 50, 475, 80]);
add_line(stsmc_sub, 'Prod_prop/1', 'Gain_k1/1');

% Super-Twisting Integral Term: -k2 * \int tanh(s/eps_bl) dt
add_block('simulink/Math Operations/Gain', [stsmc_sub '/Gain_k2'], ...
    'Gain', '-k2', 'Position', [380, 130, 420, 160]);
add_block('simulink/Continuous/Integrator', [stsmc_sub '/Integrator_w_int'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', '39.2', 'LowerSaturationLimit', '-39.2', ...
    'Position', [450, 130, 480, 160]);

add_line(stsmc_sub, 'Tanh_sign_s/1', 'Gain_k2/1');
add_line(stsmc_sub, 'Gain_k2/1', 'Integrator_w_int/1');

% Total Active STSMC Control Force: u = u1 + u2
add_block('simulink/Math Operations/Sum', [stsmc_sub '/Sum_u_total'], ...
    'Inputs', '++', 'Position', [520, 85, 545, 135]);

add_line(stsmc_sub, 'Gain_k1/1', 'Sum_u_total/1');
add_line(stsmc_sub, 'Integrator_w_int/1', 'Sum_u_total/2');
add_line(stsmc_sub, 'Sum_u_total/1', 'u_stsmc/1');

% Direct Wiring: Plant state output x_sim -> STSMC Controller -> Saturation (+/- 39.2 N) -> Plant input U
add_line(model_name, 'Open-loop Simulation/1', 'STSMC_Controller/1', 'autorouting', 'on');
add_line(model_name, 'STSMC_Controller/1', 'Saturation/1', 'autorouting', 'on');

fprintf('[3/3] Wiring loggers for automated response analysis...\n');

% Add To-Workspace Loggers
add_block('simulink/Sinks/To Workspace', [model_name '/Log_zs_ddot'], ...
    'VariableName', 'zs_ddot_sim', 'SaveFormat', 'Array', 'Position', [810, 100, 870, 120]);
add_line(model_name, 'Open-loop Simulation/2', 'Log_zs_ddot/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_susp_deflection'], ...
    'VariableName', 'susp_defl_sim', 'SaveFormat', 'Array', 'Position', [810, 160, 870, 180]);
add_line(model_name, 'Open-loop Simulation/3', 'Log_susp_deflection/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_Fc_force'], ...
    'VariableName', 'Fc_force_sim', 'SaveFormat', 'Array', 'Position', [385, 170, 445, 190]);
add_line(model_name, 'Saturation/1', 'Log_Fc_force/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_zr'], ...
    'VariableName', 'zr_sim', 'SaveFormat', 'Array', 'Position', [80, 15, 140, 35]);
add_line(model_name, 'Road Surface Generator/1', 'Log_zr/1');

save_system(model_name, fullfile(pwd, [model_name '.slx']));
close_system(model_name, 0);

fprintf('========================================================================\n');
fprintf('  PURE STSMC MODEL %s.slx CREATED SUCCESSFULLY!\n', model_name);
fprintf('========================================================================\n\n');
