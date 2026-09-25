% BUILD_STSMC_MODEL
% Programmatic Model Builder for Quanser Active Suspension with STSMC & RSI Profiles
%
% Builds Master_Active_Suspension_Simulation.slx according to stsmc_simulink_integration.md:
%   1. Clones Master_Active_Suspension_Simulation_Original.slx
%   2. Replaces Gain5 with:
%      - MATLAB Function block implementing STSMC & multi-controller selection (Passive, Classical, ACO, HS, Fixed STSMC, Adaptive STSMC)
%      - Real Simulink Integrator block for auxiliary integrator state w_int
%   3. Connects controller force into Saturation (+/- 39.2 N) -> Mux2 -> Open-Loop State-Space
%   4. Sets InitFcn = 'setup_as;' for 100% out-of-the-box auto-initialization
%   5. Sets ODE45 solver settings (StopTime=10s, RelTol=1e-4, MaxStep=1e-3)

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  BUILDING COMPLETE STSMC SIMULINK MODEL (EXACT WIRING)\n');
fprintf('========================================================================\n\n');

setup_as;

orig_model = 'Master_Active_Suspension_Simulation_Original';
model_name = 'Master_Active_Suspension_Simulation';

% Ensure original model is closed
close_system(orig_model, 0);
close_system(model_name, 0);

% Load original template
load_system(orig_model);

% Save as target model
save_system(orig_model, fullfile(pwd, [model_name '.slx']));
close_system(orig_model, 0);

load_system(model_name);
set_param(model_name, 'Lock', 'off');

fprintf('Successfully cloned base model to %s.slx\n', model_name);

% Set Model Callbacks & Simulation Parameters
set_param(model_name, 'InitFcn', 'setup_as;');
set_param(model_name, 'StopTime', '10.0');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '1e-3');

% 1. Find and remove old Gain5 block and its line to Manual Switch
gain_blk = [model_name '/Gain5'];
if getSimulinkBlockHandle(gain_blk) ~= -1
    % Find lines connected to Gain5
    port_handles = get_param(gain_blk, 'PortHandles');
    line_in  = get_param(port_handles.Inport(1), 'Line');
    line_out = get_param(port_handles.Outport(1), 'Line');
    
    if line_out ~= -1
        delete_line(line_out);
    end
    if line_in ~= -1
        delete_line(line_in);
    end
    
    delete_block(gain_blk);
    fprintf('Removed legacy Gain5 block.\n');
end

% 2. Add MATLAB Function block: STSMC_Controller
mf_blk = [model_name '/STSMC_Controller'];
if getSimulinkBlockHandle(mf_blk) == -1
    add_block('simulink/User-Defined Functions/MATLAB Function', mf_blk, ...
        'Position', [650, 240, 780, 310]);
end

% Assign exact code to MATLAB Function block
sf = sfroot;
chart = sf.find('-isa', 'Stateflow.EMChart', 'Path', mf_blk);
chart_code = sprintf([...
    'function [u, w_dot] = STSMC_Controller(x, w_int, CONTROLLER_ID, K_classical, K_aco, K_hs, c1, k1, k2, c1_adap, k1_adap, k2_adap, eps_bl)\n', ...
    '%%#codegen\n', ...
    '%% Multi-Controller Execution Block for Quanser Active Suspension\n', ...
    '%% CONTROLLER_ID 1: Passive (u = 0)\n', ...
    '%% CONTROLLER_ID 2: Classical LQR\n', ...
    '%% CONTROLLER_ID 3: ACO-Tuned LQR\n', ...
    '%% CONTROLLER_ID 4: HS-Tuned LQR\n', ...
    '%% CONTROLLER_ID 5: Fixed STSMC\n', ...
    '%% CONTROLLER_ID 6: RSI-Adaptive STSMC\n\n', ...
    'if CONTROLLER_ID == 1\n', ...
    '    u = 0.0;\n', ...
    '    w_dot = 0.0;\n', ...
    'elseif CONTROLLER_ID == 2\n', ...
    '    u = -K_classical * x;\n', ...
    '    w_dot = 0.0;\n', ...
    'elseif CONTROLLER_ID == 3\n', ...
    '    u = -K_aco * x;\n', ...
    '    w_dot = 0.0;\n', ...
    'elseif CONTROLLER_ID == 4\n', ...
    '    u = -K_hs * x;\n', ...
    '    w_dot = 0.0;\n', ...
    'elseif CONTROLLER_ID == 5\n', ...
    '    [u, w_dot] = stsmc_block(x, w_int, c1, k1, k2, eps_bl);\n', ...
    'else\n', ...
    '    [u, w_dot] = stsmc_block(x, w_int, c1_adap, k1_adap, k2_adap, eps_bl);\n', ...
    'end\n', ...
    'end\n']);
chart.Script = chart_code;
fprintf('Configured STSMC_Controller MATLAB Function block with full multi-controller logic.\n');

% Set chart parameter scopes: x and w_int are inputs, u and w_dot are outputs,
% and parameters CONTROLLER_ID, K_classical, etc., are resolved from workspace.
vars = chart.find('-isa', 'Stateflow.Data');
for i = 1:length(vars)
    vname = vars(i).Name;
    if ismember(vname, {'CONTROLLER_ID', 'K_classical', 'K_aco', 'K_hs', 'c1', 'k1', 'k2', 'c1_adap', 'k1_adap', 'k2_adap', 'eps_bl'})
        vars(i).Scope = 'Parameter';
    end
end
fprintf('Configured parameters as workspace variables (CONTROLLER_ID, K_*, c1, k1, k2, eps_bl).\n');

% 3. Add Integrator block for auxiliary integrator state w_int
int_blk = [model_name '/STSMC_Integrator'];
if getSimulinkBlockHandle(int_blk) == -1
    add_block('simulink/Continuous/Integrator', int_blk, ...
        'Position', [710, 340, 740, 370], ...
        'InitialCondition', '0.0');
end

% 4. Wire signals:
% Connect Open-loop Simulation out:1 (x_sim) -> STSMC_Controller in:1 (x)
add_line(model_name, 'Open-loop Simulation/1', 'STSMC_Controller/1', 'autorouting', 'on');

% Connect STSMC_Controller out:2 (w_dot) -> STSMC_Integrator in:1
add_line(model_name, 'STSMC_Controller/2', 'STSMC_Integrator/1', 'autorouting', 'on');

% Connect STSMC_Integrator out:1 (w_int) -> STSMC_Controller in:2 (w_int)
add_line(model_name, 'STSMC_Integrator/1', 'STSMC_Controller/2', 'autorouting', 'on');

% Connect STSMC_Controller out:1 (u) -> Manual Switch in:1
add_line(model_name, 'STSMC_Controller/1', 'Manual Switch/1', 'autorouting', 'on');

fprintf('Wired all signals: Plant State -> STSMC Controller -> Integrator Loop -> Manual Switch -> Saturation.\n');

% Save and close target model
save_system(model_name);
close_system(model_name, 0);

fprintf('\n========================================================================\n');
fprintf('  SIMULINK MODEL BUILD COMPLETE: %s.slx READY\n', model_name);
fprintf('========================================================================\n\n');
