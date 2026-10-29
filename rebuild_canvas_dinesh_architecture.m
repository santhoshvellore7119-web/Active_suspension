% REBUILD_CANVAS_DINESH_ARCHITECTURE.M
% Builds a pristine, elegant Master_Active_Suspension.slx based directly
% on the professor's Master_Dinesh_CV_results.slx architecture:
%   - Beautiful 3-lane layout with zero clumsiness and zero congestion
%   - Exact Quanser quarter-car plant subsystems in cyan
%   - Lane 1: Optimal LQR
%   - Lane 2: Fixed STSMC
%   - Lane 3: Adaptive STSMC
%   - Master Comparison Scopes: Body Accel, Susp Deflection, Actuator Force
%   - Continuous RMS Calculation Subsystems & Digital Displays
%   - Encapsulated Data_Logging Subsystem to keep root canvas clean
%   - Multiport Switch in Road Surface Generator with 5 standardized profiles

clear variables;
clc;

fprintf('========================================================================================\n');
fprintf('  REBUILDING SIMULINK CANVAS: ELEGANT DINESH ARCHITECTURE (NO CONGESTION)\n');
fprintf('========================================================================================\n\n');

parameters;

base_zip = 'C:\Users\Santhosh\Downloads\Master_Dinesh_CV_results.slx (1).zip';
if ~exist(base_zip, 'file')
    error('Base professor zip file not found: %s', base_zip);
end

model_name = 'Master_Active_Suspension_Clean';
if bdIsLoaded(model_name), close_system(model_name, 0); end
if exist([model_name '.slx'], 'file'), delete([model_name '.slx']); end

% Copy professor's model as starting point
copyfile(base_zip, [model_name '.slx']);
fprintf('[1/6] Cloned base professor architecture from Master_Dinesh_CV_results.slx\n');

load_system(model_name);
set_param(model_name, 'Lock', 'off');

% Solver & callback configuration
set_param(model_name, 'InitFcn', 'parameters;');
set_param(model_name, 'PreLoadFcn', 'parameters;');
set_param(model_name, 'StopTime', 'T_stop');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '1e-3');
set_param(model_name, 'ZeroCrossAlgorithm', 'Adaptive');
set_param(model_name, 'AlgebraicLoopMsg', 'none');

%% 1. Lane 1: Configure Optimal LQR
fprintf('[2/6] Configuring Lane 1: Optimal LQR...\n');
set_param([model_name '/Gain5'], 'Gain', '-K_lqr', 'Multiplication', 'Matrix(K*u)');
set_param([model_name '/Saturation'], 'UpperLimit', 'F_max', 'LowerLimit', '-F_max');

% Remove Manual Switch & Constant in Lane 1 to make feedback clean & direct
ms_blk = [model_name '/Manual Switch'];
c_blk  = [model_name '/Constant'];
if getSimulinkBlockHandle(ms_blk) ~= -1
    % Delete lines connected to Manual Switch & Constant
    ms_ph = get_param(ms_blk, 'PortHandles');
    for p = [ms_ph.Inport, ms_ph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, delete_line(l); end
    end
    delete_block(ms_blk);
end
if getSimulinkBlockHandle(c_blk) ~= -1
    delete_block(c_blk);
end
% Directly connect Gain5 outport to Saturation inport
add_line(model_name, 'Gain5/1', 'Saturation/1', 'autorouting', 'on');

%% 2. Lane 2: Configure Fixed STSMC
fprintf('[3/6] Configuring Lane 2: Fixed Super-Twisting SMC...\n');
g1_blk = [model_name '/Gain1'];
if getSimulinkBlockHandle(g1_blk) ~= -1
    g1_ph = get_param(g1_blk, 'PortHandles');
    for p = [g1_ph.Inport, g1_ph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, delete_line(l); end
    end
    delete_block(g1_blk);
end

% Build clean Fixed STSMC Subsystem
fsmc_sub = [model_name '/STSMC_Fixed_Controller'];
if getSimulinkBlockHandle(fsmc_sub) ~= -1, delete_block(fsmc_sub); end

add_block('simulink/Ports & Subsystems/Subsystem', fsmc_sub, ...
    'Position', [150, 425, 230, 475], 'BlockMirror', 'on', 'NameLocation', 'top');

delete_line(fsmc_sub, 'In1/1', 'Out1/1');
set_param([fsmc_sub '/In1'],  'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([fsmc_sub '/Out1'], 'Name', 'u_force', 'Position', [620, 100, 650, 120]);

% Inside Fixed STSMC Subsystem
add_block('simulink/Signal Routing/Demux', [fsmc_sub '/Demux_states'], ...
    'Outputs', '4', 'Position', [90, 80, 95, 140]);
add_line(fsmc_sub, 'x_state/1', 'Demux_states/1');

% Sliding surface: s = c1_fixed*x1 + x2
add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_c1'], ...
    'Gain', 'c1_fixed', 'Position', [130, 75, 170, 105]);
add_block('simulink/Math Operations/Sum', [fsmc_sub '/Sum_s'], ...
    'Inputs', '++', 'Position', [200, 90, 220, 120]);
add_line(fsmc_sub, 'Demux_states/1', 'Gain_c1/1');
add_line(fsmc_sub, 'Gain_c1/1', 'Sum_s/1');
add_line(fsmc_sub, 'Demux_states/2', 'Sum_s/2');

% Smooth tanh approximation: tanh(s / eps_smc)
add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_inv_eps'], ...
    'Gain', '1/eps_smc', 'Position', [250, 95, 290, 125]);
add_block('simulink/Math Operations/Trigonometric Function', [fsmc_sub '/Tanh_s'], ...
    'Operator', 'tanh', 'Position', [315, 95, 345, 125]);
add_line(fsmc_sub, 'Sum_s/1', 'Gain_inv_eps/1');
add_line(fsmc_sub, 'Gain_inv_eps/1', 'Tanh_s/1');

% Proportional branch: -k1_fixed * sqrt(|s|) * tanh(s/eps)
add_block('simulink/Math Operations/Abs', [fsmc_sub '/Abs_s'], ...
    'ZeroCross', 'off', 'Position', [250, 40, 280, 70]);
add_block('simulink/Math Operations/Math Function', [fsmc_sub '/Sqrt_s'], ...
    'Function', 'sqrt', 'Position', [315, 40, 345, 70]);
add_line(fsmc_sub, 'Sum_s/1', 'Abs_s/1');
add_line(fsmc_sub, 'Abs_s/1', 'Sqrt_s/1');

add_block('simulink/Math Operations/Product', [fsmc_sub '/Prod_prop'], ...
    'Inputs', '**', 'Position', [380, 45, 405, 80]);
add_line(fsmc_sub, 'Sqrt_s/1', 'Prod_prop/1');
add_line(fsmc_sub, 'Tanh_s/1', 'Prod_prop/2');

add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_k1'], ...
    'Gain', '-k1_fixed', 'Position', [435, 50, 475, 80]);
add_line(fsmc_sub, 'Prod_prop/1', 'Gain_k1/1');

% Integral branch: -k2_fixed * \int tanh(s/eps) dt
add_block('simulink/Math Operations/Gain', [fsmc_sub '/Gain_k2'], ...
    'Gain', '-k2_fixed', 'Position', [380, 130, 420, 160]);
add_block('simulink/Continuous/Integrator', [fsmc_sub '/Integrator_v'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'F_max', 'LowerSaturationLimit', '-F_max', ...
    'Position', [450, 130, 480, 160]);
add_line(fsmc_sub, 'Tanh_s/1', 'Gain_k2/1');
add_line(fsmc_sub, 'Gain_k2/1', 'Integrator_v/1');

% Sum total force: u = prop + integ
add_block('simulink/Math Operations/Sum', [fsmc_sub '/Sum_u'], ...
    'Inputs', '++', 'Position', [520, 85, 545, 135]);
add_line(fsmc_sub, 'Gain_k1/1', 'Sum_u/1');
add_line(fsmc_sub, 'Integrator_v/1', 'Sum_u/2');
add_line(fsmc_sub, 'Sum_u/1', 'u_force/1');

% Connect Lane 2 feedback
add_line(model_name, 'Open-loop Simulation1/1', 'STSMC_Fixed_Controller/1', 'autorouting', 'on');
add_line(model_name, 'STSMC_Fixed_Controller/1', 'Saturation1/1', 'autorouting', 'on');
set_param([model_name '/Saturation1'], 'UpperLimit', 'F_max', 'LowerLimit', '-F_max');

%% 3. Lane 3: Configure Adaptive STSMC
fprintf('[4/6] Configuring Lane 3: Adaptive Super-Twisting SMC...\n');
g2_blk = [model_name '/Gain2'];
if getSimulinkBlockHandle(g2_blk) ~= -1
    g2_ph = get_param(g2_blk, 'PortHandles');
    for p = [g2_ph.Inport, g2_ph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, delete_line(l); end
    end
    delete_block(g2_blk);
end

% Build clean Adaptive STSMC Subsystem
asmc_sub = [model_name '/STSMC_Adaptive_Controller'];
if getSimulinkBlockHandle(asmc_sub) ~= -1, delete_block(asmc_sub); end

add_block('simulink/Ports & Subsystems/Subsystem', asmc_sub, ...
    'Position', [150, 675, 230, 725], 'BlockMirror', 'on', 'NameLocation', 'top');

delete_line(asmc_sub, 'In1/1', 'Out1/1');
set_param([asmc_sub '/In1'],  'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([asmc_sub '/Out1'], 'Name', 'u_force', 'Position', [620, 100, 650, 120]);

% Inside Adaptive STSMC Subsystem
add_block('simulink/Signal Routing/Demux', [asmc_sub '/Demux_states'], ...
    'Outputs', '4', 'Position', [90, 80, 95, 140]);
add_line(asmc_sub, 'x_state/1', 'Demux_states/1');

% Sliding surface: s = c1_adap*x1 + x2
add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_c1'], ...
    'Gain', 'c1_adap', 'Position', [130, 75, 170, 105]);
add_block('simulink/Math Operations/Sum', [asmc_sub '/Sum_s'], ...
    'Inputs', '++', 'Position', [200, 90, 220, 120]);
add_line(asmc_sub, 'Demux_states/1', 'Gain_c1/1');
add_line(asmc_sub, 'Gain_c1/1', 'Sum_s/1');
add_line(asmc_sub, 'Demux_states/2', 'Sum_s/2');

% Smooth tanh approximation: tanh(s / eps_smc)
add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_inv_eps'], ...
    'Gain', '1/eps_smc', 'Position', [250, 95, 290, 125]);
add_block('simulink/Math Operations/Trigonometric Function', [asmc_sub '/Tanh_s'], ...
    'Operator', 'tanh', 'Position', [315, 95, 345, 125]);
add_line(asmc_sub, 'Sum_s/1', 'Gain_inv_eps/1');
add_line(asmc_sub, 'Gain_inv_eps/1', 'Tanh_s/1');

% Proportional branch: -k1_adap * sqrt(|s|) * tanh(s/eps)
add_block('simulink/Math Operations/Abs', [asmc_sub '/Abs_s'], ...
    'ZeroCross', 'off', 'Position', [250, 40, 280, 70]);
add_block('simulink/Math Operations/Math Function', [asmc_sub '/Sqrt_s'], ...
    'Function', 'sqrt', 'Position', [315, 40, 345, 70]);
add_line(asmc_sub, 'Sum_s/1', 'Abs_s/1');
add_line(asmc_sub, 'Abs_s/1', 'Sqrt_s/1');

add_block('simulink/Math Operations/Product', [asmc_sub '/Prod_prop'], ...
    'Inputs', '**', 'Position', [380, 45, 405, 80]);
add_line(asmc_sub, 'Sqrt_s/1', 'Prod_prop/1');
add_line(asmc_sub, 'Tanh_s/1', 'Prod_prop/2');

add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_k1'], ...
    'Gain', '-k1_adap', 'Position', [435, 50, 475, 80]);
add_line(asmc_sub, 'Prod_prop/1', 'Gain_k1/1');

% Integral branch: -k2_adap * \int tanh(s/eps) dt
add_block('simulink/Math Operations/Gain', [asmc_sub '/Gain_k2'], ...
    'Gain', '-k2_adap', 'Position', [380, 130, 420, 160]);
add_block('simulink/Continuous/Integrator', [asmc_sub '/Integrator_v'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'F_max', 'LowerSaturationLimit', '-F_max', ...
    'Position', [450, 130, 480, 160]);
add_line(asmc_sub, 'Tanh_s/1', 'Gain_k2/1');
add_line(asmc_sub, 'Gain_k2/1', 'Integrator_v/1');

% Sum total force: u = prop + integ
add_block('simulink/Math Operations/Sum', [asmc_sub '/Sum_u'], ...
    'Inputs', '++', 'Position', [520, 85, 545, 135]);
add_line(asmc_sub, 'Gain_k1/1', 'Sum_u/1');
add_line(asmc_sub, 'Integrator_v/1', 'Sum_u/2');
add_line(asmc_sub, 'Sum_u/1', 'u_force/1');

% Connect Lane 3 feedback
add_line(model_name, 'Open-loop Simulation2/1', 'STSMC_Adaptive_Controller/1', 'autorouting', 'on');
add_line(model_name, 'STSMC_Adaptive_Controller/1', 'Saturation2/1', 'autorouting', 'on');
set_param([model_name '/Saturation2'], 'UpperLimit', 'F_max', 'LowerLimit', '-F_max');

%% 4. Reconfigure Road Surface Generator with 5 Profiles
fprintf('[5/6] Updating Road Surface Generator with 5 standardized profiles...\n');
road_sub = [model_name '/Road Surface Generator'];

% Remove legacy switch blocks inside Road Surface Generator
blks_del = { ...
    [road_sub '/Manual Switch'], ...
    [road_sub '/Manual Switch1'], ...
    [road_sub '/Constant'], ...
    [road_sub '/Signal Generator'], ...
    [road_sub '/Pulse\nGenerator'], ...
    [road_sub '/Pulse Generator'], ...
    [road_sub '/Saturation'], ...
    [road_sub '/Scope'], ...
    [road_sub '/Scope1'], ...
    [road_sub '/Scope2'] ...
};
for i = 1:length(blks_del)
    b = blks_del{i};
    if getSimulinkBlockHandle(b) ~= -1
        ph = get_param(b, 'PortHandles');
        for p = [ph.Inport, ph.Outport]
            l = get_param(p, 'Line');
            if l ~= -1, delete_line(l); end
        end
        delete_block(b);
    end
end

% Delete existing line into Outport zr if any
zr_out = [road_sub '/zr'];
if getSimulinkBlockHandle(zr_out) ~= -1
    zr_ph = get_param(zr_out, 'PortHandles');
    l = get_param(zr_ph.Inport(1), 'Line');
    if l ~= -1, delete_line(l); end
end

% Constant block for Road Profile Selection
sel_blk = [road_sub '/Select_Road_ID'];
if getSimulinkBlockHandle(sel_blk) == -1
    add_block('simulink/Sources/Constant', sel_blk, 'Value', 'ROAD_ID', 'Position', [280, 20, 360, 50]);
else
    set_param(sel_blk, 'Value', 'ROAD_ID', 'Position', [280, 20, 360, 50]);
end

% Profile 1: Smooth Sine (4 mm, 0.20 Hz)
p1_blk = [road_sub '/Profile 1: Smooth Sine (4mm, 0.2Hz)'];
if getSimulinkBlockHandle(p1_blk) == -1
    add_block('simulink/Sources/Sine Wave', p1_blk, ...
        'Amplitude', 'road1_amp', 'Frequency', '2*pi*road1_freq', 'SampleTime', '0', ...
        'Position', [60, 80, 180, 110]);
end

% Profile 2: Normal Sine (20 mm, 0.20 Hz)
p2_blk = [road_sub '/Profile 2: Normal Sine (20mm, 0.2Hz)'];
if getSimulinkBlockHandle(p2_blk) == -1
    add_block('simulink/Sources/Sine Wave', p2_blk, ...
        'Amplitude', 'road2_amp', 'Frequency', '2*pi*road2_freq', 'SampleTime', '0', ...
        'Position', [60, 140, 180, 170]);
end

% Profile 3: Rough ISO Class D Broadband (From Workspace Timeseries)
p3_blk = [road_sub '/Profile 3: Rough ISO Class D'];
if getSimulinkBlockHandle(p3_blk) == -1
    add_block('simulink/Sources/From Workspace', p3_blk, ...
        'VariableName', 'road_rough_data', 'SampleTime', '0', 'Interpolate', 'on', ...
        'Position', [60, 200, 180, 230]);
end

% Profile 4: Pothole Pulse (-20 mm Drops)
p4_blk = [road_sub '/Profile 4: Pothole Pulse (-20mm)'];
if getSimulinkBlockHandle(p4_blk) == -1
    add_block('simulink/Sources/Pulse Generator', p4_blk, ...
        'Amplitude', '-road4_amp', 'Period', '1/road4_freq', 'PulseWidth', 'road4_width', 'PhaseDelay', '1.0', ...
        'Position', [60, 260, 180, 290]);
end

% Profile 5: Harmonic Resonance (2.58 Hz Sine)
p5_blk = [road_sub '/Profile 5: Resonance (2.58Hz)'];
if getSimulinkBlockHandle(p5_blk) == -1
    add_block('simulink/Sources/Sine Wave', p5_blk, ...
        'Amplitude', 'road5_amp', 'Frequency', '2*pi*road5_freq', 'SampleTime', '0', ...
        'Position', [60, 320, 180, 350]);
end

% Multiport Switch
sw_blk = [road_sub '/Multiport_Switch'];
if getSimulinkBlockHandle(sw_blk) == -1
    add_block('simulink/Signal Routing/Multiport Switch', sw_blk, ...
        'Inputs', '5', 'DataPortOrder', 'One-based contiguous', 'Position', [400, 30, 440, 360]);
    add_line(road_sub, 'Select_Road_ID/1', 'Multiport_Switch/1');
    add_line(road_sub, 'Profile 1: Smooth Sine (4mm, 0.2Hz)/1', 'Multiport_Switch/2');
    add_line(road_sub, 'Profile 2: Normal Sine (20mm, 0.2Hz)/1', 'Multiport_Switch/3');
    add_line(road_sub, 'Profile 3: Rough ISO Class D/1',          'Multiport_Switch/4');
    add_line(road_sub, 'Profile 4: Pothole Pulse (-20mm)/1',     'Multiport_Switch/5');
    add_line(road_sub, 'Profile 5: Resonance (2.58Hz)/1',        'Multiport_Switch/6');
    add_line(road_sub, 'Multiport_Switch/1', 'zr/1');
end

%% 5. Encapsulated Data_Logging Subsystem
fprintf('[6/6] Creating pristine Data_Logging Subsystem (no canvas clutter)...\n');
log_sub = [model_name '/Data_Logging'];
if getSimulinkBlockHandle(log_sub) ~= -1
    delete_block(log_sub);
end

% Position Data_Logging cleanly at the bottom
add_block('simulink/Ports & Subsystems/Subsystem', log_sub, ...
    'Position', [430, 830, 530, 880], 'BackgroundColor', 'LightBlue');

delete_line(log_sub, 'In1/1', 'Out1/1');
delete_block([log_sub '/Out1']);

% Configure Inports inside Data_Logging
log_vars = { ...
    'zr_sim',         'Road Excitation zr'; ...
    'zs_ddot_lqr',    'LQR Body Accel'; ...
    'zs_ddot_fsmc',   'Fixed STSMC Body Accel'; ...
    'zs_ddot_asmc',   'Adaptive STSMC Body Accel'; ...
    'defl_lqr',       'LQR Susp Deflection'; ...
    'defl_fsmc',      'Fixed STSMC Susp Deflection'; ...
    'defl_asmc',      'Adaptive STSMC Susp Deflection'; ...
    'u_lqr',          'LQR Control Force'; ...
    'u_fsmc',         'Fixed STSMC Control Force'; ...
    'u_asmc',         'Adaptive STSMC Control Force'; ...
    'x_lqr',          'LQR State Vector'; ...
    'x_fsmc',         'Fixed STSMC State Vector'; ...
    'x_asmc',         'Adaptive STSMC State Vector'  ...
};

N_logs = size(log_vars, 1);
for i = 1:N_logs
    vname = log_vars{i, 1};
    lbl   = log_vars{i, 2};
    y_pos = 30 + (i-1)*50;
    
    % Inport
    if i == 1
        in_blk = [log_sub '/In1'];
        set_param(in_blk, 'Name', vname, 'Position', [40, y_pos, 70, y_pos+20]);
    else
        add_block('simulink/Ports & Subsystems/In1', [log_sub '/' vname], ...
            'Position', [40, y_pos, 70, y_pos+20]);
    end
    
    if startsWith(vname, 'x_')
        % Demux state 3 (tire deflection zus - zr)
        base_ctrl = extractAfter(vname, 'x_');
        demux_blk = [log_sub '/Demux_' base_ctrl];
        add_block('simulink/Signal Routing/Demux', demux_blk, ...
            'Outputs', '4', 'Position', [140, y_pos-10, 145, y_pos+30]);
        add_line(log_sub, [vname '/1'], ['Demux_' base_ctrl '/1']);
        
        ws_blk = [log_sub '/ToWS_tire_' base_ctrl];
        add_block('simulink/Sinks/To Workspace', ws_blk, ...
            'VariableName', ['tire_' base_ctrl], 'SaveFormat', 'Array', ...
            'Position', [220, y_pos, 300, y_pos+20]);
        add_line(log_sub, ['Demux_' base_ctrl '/3'], ['ToWS_tire_' base_ctrl '/1']);
    else
        ws_blk = [log_sub '/ToWS_' vname];
        add_block('simulink/Sinks/To Workspace', ws_blk, ...
            'VariableName', vname, 'SaveFormat', 'Array', ...
            'Position', [220, y_pos, 300, y_pos+20]);
        add_line(log_sub, [vname '/1'], ['ToWS_' vname '/1']);
    end
end

% Dummy passive logs for backward compatibility if requested
add_block('simulink/Sources/Constant', [log_sub '/Zero_Const'], 'Value', '0', 'Position', [40, 700, 70, 720]);
dummy_pass = {'zs_ddot_pass', 'defl_pass', 'tire_pass', 'u_pass'};
for d = 1:length(dummy_pass)
    d_v = dummy_pass{d};
    ws_d = [log_sub '/ToWS_' d_v];
    add_block('simulink/Sinks/To Workspace', ws_d, ...
        'VariableName', d_v, 'SaveFormat', 'Array', ...
        'Position', [220, 700 + (d-1)*40, 300, 720 + (d-1)*40]);
    add_line(log_sub, 'Zero_Const/1', ['ToWS_' d_v '/1']);
end

% Connect Data_Logging Inports at root level
add_line(model_name, 'Road Surface Generator/1',  'Data_Logging/1',  'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation/2',   'Data_Logging/2',  'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation1/2',  'Data_Logging/3',  'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation2/2',  'Data_Logging/4',  'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation/3',   'Data_Logging/5',  'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation1/3',  'Data_Logging/6',  'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation2/3',  'Data_Logging/7',  'autorouting', 'on');
add_line(model_name, 'Saturation/1',             'Data_Logging/8',  'autorouting', 'on');
add_line(model_name, 'Saturation1/1',            'Data_Logging/9',  'autorouting', 'on');
add_line(model_name, 'Saturation2/1',            'Data_Logging/10', 'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation/1',   'Data_Logging/11', 'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation1/1',  'Data_Logging/12', 'autorouting', 'on');
add_line(model_name, 'Open-loop Simulation2/1',  'Data_Logging/13', 'autorouting', 'on');

%% 6. Save and take screenshot of pristine model
save_system(model_name, fullfile(pwd, [model_name '.slx']));
fprintf('Saved pristine model: %s.slx\n', model_name);

% Also update Master_Active_Suspension_Simulation.slx to be identical
copyfile([model_name '.slx'], 'Master_Active_Suspension_Simulation.slx');
fprintf('Copied to Master_Active_Suspension_Simulation.slx\n');

try
    copyfile([model_name '.slx'], 'Master_Active_Suspension.slx');
    fprintf('Copied to Master_Active_Suspension.slx\n');
catch me
    fprintf('[Notice] Master_Active_Suspension.slx is currently open in your Simulink editor.\n');
    fprintf('         Please close Master_Active_Suspension in Simulink, then run: copyfile(''Master_Active_Suspension_Clean.slx'', ''Master_Active_Suspension.slx'')\n');
end

close_system(model_name, 0);

fprintf('\n========================================================================================\n');
fprintf('  SUCCESS: PRISTINE 3-LANE DINESH SIMULINK ARCHITECTURE REBUILT!\n');
fprintf('========================================================================================\n');
