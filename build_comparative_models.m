% BUILD_COMPARATIVE_MODELS
% Programmatically constructs Master_Active_Suspension_Simulation.slx
% Following the exact comparative format of Master_Dinesh_CV_results.slx:
%
% Architecture:
%   - Shared Road Surface Generator with 6 Standardized Road Profiles (Multiport Switch)
%   - Shared 2nd-Order Derivative Filter (Differentiator -> zr_dot)
%   - THREE PARALLEL QUANSER PLANTS running simultaneously side-by-side:
%       * Branch 1: Passive Baseline (with Manual Switch to Classical LQR)
%       * Branch 2: LQR Controller (Optimal ACO-Tuned LQR)
%       * Branch 3: SMC Controller (Super-Twisting Sliding Mode Control - STSMC)
%   - Continuous In-Simulink RMS Calculation Subsystems & Canvas Digital Displays:
%       * RMS Suspension Deflection for Passive, LQR, and STSMC (in mm)
%       * RMS Body Acceleration for Passive, LQR, and STSMC (in m/s^2)
%   - Multi-Trace Comparison Scopes:
%       * Body Acceleration Comparison (zs_ddot): Passive vs LQR vs STSMC
%       * Suspension Deflection Comparison: Passive vs LQR vs STSMC
%       * Control Force Comparison (Fc): Passive vs LQR vs STSMC
%   - Individual Plant Scopes for deep-dive tracking inspection
%   - Base Workspace Loggers for automated data export

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  BUILDING UNIFIED PARALLEL COMPARATIVE SIMULINK MODEL (PASSIVE vs LQR vs STSMC)\n');
fprintf('  Following Exact Format of Master_Dinesh_CV_results.slx\n');
fprintf('========================================================================\n\n');

setup_as;

model_name = 'Master_Active_Suspension_Simulation';
dinesh_zip = 'C:\Users\Santhosh\Downloads\Master_Dinesh_CV_results.slx (1).zip';

% Close models if already open
if bdIsLoaded(model_name), close_system(model_name, 0); end

% Backup existing single-plant model if present
if exist([model_name '.slx'], 'file')
    copyfile([model_name '.slx'], [model_name '_SinglePlant_Backup.slx']);
    fprintf('[1/7] Backed up previous model to %s_SinglePlant_Backup.slx\n', model_name);
end

% Clone Dinesh comparative model as base to preserve exact placement and styling
copyfile(dinesh_zip, [model_name '.slx']);
fprintf('[2/7] Cloned Dinesh comparative architecture as target: %s.slx\n', model_name);

% Load target model
load_system(model_name);
set_param(model_name, 'Lock', 'off');

% Set solver and callbacks
set_param(model_name, 'InitFcn', 'setup_as;');
set_param(model_name, 'PreLoadFcn', 'setup_as;');
set_param(model_name, 'StopTime', '10.0');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '1e-3');
set_param(model_name, 'AlgebraicLoopMsg', 'none');
set_param(model_name, 'ZeroCrossAlgorithm', 'Adaptive');
set_param(model_name, 'MaxConsecutiveZCs', '10000');

fprintf('[3/7] Reconfiguring Branch 3 from HS-LQR to Pure Native STSMC Controller...\n');

% Branch 3 Controller: Delete Gain2 (HS-LQR) and wire STSMC Subsystem
gain2_blk = [model_name '/Gain2'];
if getSimulinkBlockHandle(gain2_blk) ~= -1
    g2_ph = get_param(gain2_blk, 'PortHandles');
    for p = [g2_ph.Inport, g2_ph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, delete_line(l); end
    end
    delete_block(gain2_blk);
end

% Build Native STSMC Subsystem in Branch 3
stsmc_sub = [model_name '/STSMC_Controller'];
if getSimulinkBlockHandle(stsmc_sub) ~= -1
    delete_block(stsmc_sub);
end

add_block('simulink/Ports & Subsystems/Subsystem', stsmc_sub, ...
    'Position', [150, 680, 230, 730], 'BlockMirror', 'on', 'NameLocation', 'top');

delete_line(stsmc_sub, 'In1/1', 'Out1/1');
set_param([stsmc_sub '/In1'], 'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([stsmc_sub '/Out1'], 'Name', 'u_stsmc', 'Position', [620, 100, 650, 120]);

% Inside STSMC Subsystem: Demux states [x1; x2; x3; x4]
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

% Super-Twisting Proportional Branch: -k1 * sqrt(|s|) * tanh(s / eps_bl)
add_block('simulink/Math Operations/Abs', [stsmc_sub '/Abs_s'], ...
    'ZeroCross', 'off', 'Position', [250, 40, 280, 70]);
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

% Super-Twisting Integral Branch: -k2 * \int tanh(s / eps_bl) dt
add_block('simulink/Math Operations/Gain', [stsmc_sub '/Gain_k2'], ...
    'Gain', '-k2', 'Position', [380, 130, 420, 160]);
add_block('simulink/Continuous/Integrator', [stsmc_sub '/Integrator_w_int'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', '39.2', 'LowerSaturationLimit', '-39.2', ...
    'Position', [450, 130, 480, 160]);

add_line(stsmc_sub, 'Tanh_sign_s/1', 'Gain_k2/1');
add_line(stsmc_sub, 'Gain_k2/1', 'Integrator_w_int/1');

% Total STSMC Control Force
add_block('simulink/Math Operations/Sum', [stsmc_sub '/Sum_u_total'], ...
    'Inputs', '++', 'Position', [520, 85, 545, 135]);
add_line(stsmc_sub, 'Gain_k1/1', 'Sum_u_total/1');
add_line(stsmc_sub, 'Integrator_w_int/1', 'Sum_u_total/2');
add_line(stsmc_sub, 'Sum_u_total/1', 'u_stsmc/1');

% Wire STSMC Subsystem in Branch 3
% Plant 3 outport 1 (x_sim) -> STSMC Controller Inport 1
add_line(model_name, 'Open-loop Simulation2/1', 'STSMC_Controller/1', 'autorouting', 'on');
% STSMC Controller Outport 1 -> Saturation2 Inport 1
add_line(model_name, 'STSMC_Controller/1', 'Saturation2/1', 'autorouting', 'on');

fprintf('[4/7] Reconfiguring Branch 1 (Classical/Passive) & Branch 2 (Optimal ACO-LQR)...\n');

% Branch 1: Classical LQR Gain (Gain5) & Manual Switch (0 = Passive, 1 = Classical LQR)
set_param([model_name '/Gain5'], 'Gain', '-K_classical');
set_param([model_name '/Constant'], 'Value', '0');     % Zero force for pure passive baseline
set_param([model_name '/Manual Switch'], 'sw', '0');   % Default to Constant 0 (Pure Passive Baseline)

% Branch 2: Optimal ACO-Tuned LQR Gain (Gain1)
set_param([model_name '/Gain1'], 'Gain', '-K_aco');

% Update Actuator Saturation limits strictly to +/- 39.2 N
set_param([model_name '/Saturation'],  'UpperLimit', 'Fc_max', 'LowerLimit', 'Fc_min');
set_param([model_name '/Saturation1'], 'UpperLimit', 'Fc_max', 'LowerLimit', 'Fc_min');
set_param([model_name '/Saturation2'], 'UpperLimit', 'Fc_max', 'LowerLimit', 'Fc_min');

fprintf('[5/7] Reconfiguring 6-Road Profile Multiport Switch inside Road Surface Generator...\n');

% Configure Road Surface Generator with all 6 Standardized Profiles
road_sub = [model_name '/Road Surface Generator'];

% Delete obsolete blocks inside Road Surface Generator if any
blks_to_del = { ...
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

for i = 1:length(blks_to_del)
    b = blks_to_del{i};
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

% Road Number Selector Block
selector_blk = [road_sub '/Enter Road Number (1 to 6)'];
if getSimulinkBlockHandle(selector_blk) == -1
    add_block('simulink/Sources/Constant', selector_blk, ...
        'Value', 'ROAD_PROFILE_ID', 'Position', [320, 20, 420, 50]);
else
    set_param(selector_blk, 'Value', 'ROAD_PROFILE_ID', 'Position', [320, 20, 420, 50]);
end

% Profile 1: Original Quanser Harmonics Generator
harm_blk = [road_sub '/ Harmonics Generator'];
if getSimulinkBlockHandle(harm_blk) ~= -1
    set_param(harm_blk, 'Position', [120, 80, 220, 130]);
end

% Profile 2: Smooth Undulating Highway
p2_blk = [road_sub '/Profile 2: Smooth Highway Sine (4 mm, 0.2 Hz)'];
if getSimulinkBlockHandle(p2_blk) == -1
    add_block('simulink/Sources/Sine Wave', p2_blk, ...
        'Amplitude', 'road_smooth_amp', 'Frequency', '2*pi*road_smooth_freq', 'SampleTime', '0', ...
        'Position', [120, 160, 220, 200]);
end

% Profile 3: Severe Discrete Potholes
p3_blk = [road_sub '/Profile 3: Severe Pothole Cavities (-20 mm)'];
if getSimulinkBlockHandle(p3_blk) == -1
    add_block('simulink/Sources/Pulse Generator', p3_blk, ...
        'Amplitude', '-road_pothole_amp', 'Period', '1/road_pothole_freq', ...
        'PulseWidth', 'road_pothole_width', 'PhaseDelay', '0.2', ...
        'Position', [120, 230, 220, 270]);
end

% Profile 4: Triangular Ramp Obstacle & Speed Bump
p4_blk = [road_sub '/Profile 4: Triangular Ramp & Speed Bump (+20 mm)'];
if getSimulinkBlockHandle(p4_blk) == -1
    add_block('simulink/Sources/Repeating Sequence', p4_blk, ...
        'rep_seq_t', '[0, 0.5, 1.5, 1.52, 4.0]', 'rep_seq_y', '[0, 0.020, 0.020, 0, 0]', ...
        'Position', [120, 300, 220, 340]);
end

% Profile 5: Real-World Stochastic Rough Terrain + Shocks
p5_blk = [road_sub '/Profile 5: Stochastic Rough Terrain + Shocks'];
if getSimulinkBlockHandle(p5_blk) == -1
    add_block('simulink/Sources/Sine Wave', p5_blk, ...
        'Amplitude', 'road_rough_amp', 'Frequency', '2*pi*4.5', 'SampleTime', '0', ...
        'Position', [120, 370, 220, 410]);
end

% Profile 6: Harmonic Corrugated Resonance Road
p6_blk = [road_sub '/Profile 6: Harmonic Resonance Corrugation (2.58 Hz)'];
if getSimulinkBlockHandle(p6_blk) == -1
    add_block('simulink/Sources/Sine Wave', p6_blk, ...
        'Amplitude', 'road_res_amp', 'Frequency', '2*pi*road_res_freq', 'SampleTime', '0', ...
        'Position', [120, 440, 220, 480]);
end

% Multiport Switch to select between all 6 profiles
mux_blk = [road_sub '/Road_Profile_Mux_Switch'];
if getSimulinkBlockHandle(mux_blk) == -1
    add_block('simulink/Signal Routing/Multiport Switch', mux_blk, ...
        'Inputs', '6', 'DataPortOrder', 'One-based contiguous', ...
        'Position', [480, 50, 520, 510]);
else
    mph = get_param(mux_blk, 'PortHandles');
    for p = [mph.Inport, mph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1, try delete_line(l); catch, end; end
    end
end

% Wire Multiport Switch
add_line(road_sub, 'Enter Road Number (1 to 6)/1', 'Road_Profile_Mux_Switch/1', 'autorouting', 'on');
add_line(road_sub, ' Harmonics Generator/1', 'Road_Profile_Mux_Switch/2', 'autorouting', 'on');
add_line(road_sub, 'Profile 2: Smooth Highway Sine (4 mm, 0.2 Hz)/1', 'Road_Profile_Mux_Switch/3', 'autorouting', 'on');
add_line(road_sub, 'Profile 3: Severe Pothole Cavities (-20 mm)/1', 'Road_Profile_Mux_Switch/4', 'autorouting', 'on');
add_line(road_sub, 'Profile 4: Triangular Ramp & Speed Bump (+20 mm)/1', 'Road_Profile_Mux_Switch/5', 'autorouting', 'on');
add_line(road_sub, 'Profile 5: Stochastic Rough Terrain + Shocks/1', 'Road_Profile_Mux_Switch/6', 'autorouting', 'on');
add_line(road_sub, 'Profile 6: Harmonic Resonance Corrugation (2.58 Hz)/1', 'Road_Profile_Mux_Switch/7', 'autorouting', 'on');

set_param(zr_out, 'Position', [620, 270, 650, 290]);
add_line(road_sub, 'Road_Profile_Mux_Switch/1', 'zr/1', 'autorouting', 'on');

fprintf('[6/7] Configuring Continuous RMS Calculation Subsystems and Live Canvas Displays...\n');

% Function to create or update an RMS calculation subsystem
% Inputs: block_path, in_name, out_name, pos
create_rms_sub = @(sub_path, pos) ...
    [ ...
    ];

% 6a. Rename existing deflection RMS blocks with clear labels and ensure non-negative integration
set_param([model_name '/Subsystem'],  'Name', 'RMS_Defl_Passive');
set_param([model_name '/Subsystem1'], 'Name', 'RMS_Defl_LQR');
set_param([model_name '/Subsystem2'], 'Name', 'RMS_Defl_STSMC');

% Prevent negative epsilon from numerical roundoff in RMS Integrators
defl_subs = {'RMS_Defl_Passive', 'RMS_Defl_LQR', 'RMS_Defl_STSMC'};
for ds = defl_subs
    int_blk = [model_name '/' ds{1} '/Integrator'];
    if getSimulinkBlockHandle(int_blk) ~= -1
        set_param(int_blk, 'LimitOutput', 'on', 'LowerSaturationLimit', '0');
    end
end

% Scale to mm with Gain blocks before displays:
g_mm1 = [model_name '/Gain_m2mm_Passive'];
if getSimulinkBlockHandle(g_mm1) == -1
    add_block('simulink/Math Operations/Gain', g_mm1, 'Gain', '1000', 'Position', [1240, 442, 1270, 468]);
    l = get_param(get_param([model_name '/Display'], 'PortHandles').Inport(1), 'Line');
    if l ~= -1, delete_line(l); end
    add_line(model_name, 'RMS_Defl_Passive/1', 'Gain_m2mm_Passive/1');
    add_line(model_name, 'Gain_m2mm_Passive/1', 'Display/1');
end

g_mm2 = [model_name '/Gain_m2mm_LQR'];
if getSimulinkBlockHandle(g_mm2) == -1
    add_block('simulink/Math Operations/Gain', g_mm2, 'Gain', '1000', 'Position', [1250, 527, 1280, 553]);
    l = get_param(get_param([model_name '/Display1'], 'PortHandles').Inport(1), 'Line');
    if l ~= -1, delete_line(l); end
    add_line(model_name, 'RMS_Defl_LQR/1', 'Gain_m2mm_LQR/1');
    add_line(model_name, 'Gain_m2mm_LQR/1', 'Display1/1');
end

g_mm3 = [model_name '/Gain_m2mm_STSMC'];
if getSimulinkBlockHandle(g_mm3) == -1
    add_block('simulink/Math Operations/Gain', g_mm3, 'Gain', '1000', 'Position', [1250, 612, 1280, 638]);
    l = get_param(get_param([model_name '/Display2'], 'PortHandles').Inport(1), 'Line');
    if l ~= -1, delete_line(l); end
    add_line(model_name, 'RMS_Defl_STSMC/1', 'Gain_m2mm_STSMC/1');
    add_line(model_name, 'Gain_m2mm_STSMC/1', 'Display2/1');
end

% Set display names and titles
set_param([model_name '/Display'],  'Name', 'Display_RMS_Defl_Passive_mm');
set_param([model_name '/Display1'], 'Name', 'Display_RMS_Defl_LQR_mm');
set_param([model_name '/Display2'], 'Name', 'Display_RMS_Defl_STSMC_mm');

% 6b. Add Continuous RMS Acceleration Subsystems & Displays
build_rms_block = @(sub_name) ...
    eval([ ...
        'add_block(''simulink/Ports & Subsystems/Subsystem'', sub_name); ' ...
        'delete_line(sub_name, ''In1/1'', ''Out1/1''); ' ...
        'set_param([sub_name ''/In1''], ''Name'', ''acc_in'', ''Position'', [20, 35, 50, 50]); ' ...
        'set_param([sub_name ''/Out1''], ''Name'', ''rms_acc'', ''Position'', [380, 35, 410, 50]); ' ...
        'add_block(''simulink/Math Operations/Math Function'', [sub_name ''/Square''], ''Function'', ''square'', ''Position'', [80, 25, 110, 60]); ' ...
        'add_block(''simulink/Continuous/Integrator'', [sub_name ''/Integrator''], ''Position'', [145, 25, 175, 60]); ' ...
        'add_block(''simulink/Math Operations/Gain'', [sub_name ''/Gain_1_T''], ''Gain'', ''1/10'', ''Position'', [215, 25, 255, 60]); ' ...
        'add_block(''simulink/Math Operations/Math Function'', [sub_name ''/Sqrt''], ''Function'', ''sqrt'', ''Position'', [295, 25, 335, 60]); ' ...
        'add_line(sub_name, ''acc_in/1'', ''Square/1''); ' ...
        'add_line(sub_name, ''Square/1'', ''Integrator/1''); ' ...
        'add_line(sub_name, ''Integrator/1'', ''Gain_1_T/1''); ' ...
        'add_line(sub_name, ''Gain_1_T/1'', ''Sqrt/1''); ' ...
        'add_line(sub_name, ''Sqrt/1'', ''rms_acc/1'');' ...
    ]);

rms_acc1 = [model_name '/RMS_Accel_Passive'];
if getSimulinkBlockHandle(rms_acc1) == -1
    add_block('simulink/Ports & Subsystems/Subsystem', rms_acc1, 'Position', [1205, 475, 1235, 505]);
    delete_line(rms_acc1, 'In1/1', 'Out1/1');
    set_param([rms_acc1 '/In1'], 'Name', 'acc_in', 'Position', [20, 35, 50, 50]);
    set_param([rms_acc1 '/Out1'], 'Name', 'rms_acc', 'Position', [380, 35, 410, 50]);
    add_block('simulink/Math Operations/Math Function', [rms_acc1 '/Square'], 'Operator', 'square', 'Position', [80, 25, 110, 60]);
    add_block('simulink/Continuous/Integrator', [rms_acc1 '/Integrator'], ...
        'LimitOutput', 'on', 'LowerSaturationLimit', '0', 'Position', [145, 25, 175, 60]);
    add_block('simulink/Math Operations/Gain', [rms_acc1 '/Gain_1_T'], 'Gain', '1/10', 'Position', [215, 25, 255, 60]);
    add_block('simulink/Math Operations/Sqrt', [rms_acc1 '/Sqrt'], 'Position', [295, 25, 335, 60]);
    add_line(rms_acc1, 'acc_in/1', 'Square/1');
    add_line(rms_acc1, 'Square/1', 'Integrator/1');
    add_line(rms_acc1, 'Integrator/1', 'Gain_1_T/1');
    add_line(rms_acc1, 'Gain_1_T/1', 'Sqrt/1');
    add_line(rms_acc1, 'Sqrt/1', 'rms_acc/1');

    add_block('simulink/Sinks/Display', [model_name '/Display_RMS_Accel_Passive_ms2'], ...
        'Position', [1285, 475, 1375, 505]);
    add_line(model_name, 'Open-loop Simulation/2', 'RMS_Accel_Passive/1', 'autorouting', 'on');
    add_line(model_name, 'RMS_Accel_Passive/1', 'Display_RMS_Accel_Passive_ms2/1');
end

rms_acc2 = [model_name '/RMS_Accel_LQR'];
if getSimulinkBlockHandle(rms_acc2) == -1
    add_block('simulink/Ports & Subsystems/Subsystem', rms_acc2, 'Position', [1205, 560, 1235, 590]);
    delete_line(rms_acc2, 'In1/1', 'Out1/1');
    set_param([rms_acc2 '/In1'], 'Name', 'acc_in', 'Position', [20, 35, 50, 50]);
    set_param([rms_acc2 '/Out1'], 'Name', 'rms_acc', 'Position', [380, 35, 410, 50]);
    add_block('simulink/Math Operations/Math Function', [rms_acc2 '/Square'], 'Operator', 'square', 'Position', [80, 25, 110, 60]);
    add_block('simulink/Continuous/Integrator', [rms_acc2 '/Integrator'], ...
        'LimitOutput', 'on', 'LowerSaturationLimit', '0', 'Position', [145, 25, 175, 60]);
    add_block('simulink/Math Operations/Gain', [rms_acc2 '/Gain_1_T'], 'Gain', '1/10', 'Position', [215, 25, 255, 60]);
    add_block('simulink/Math Operations/Sqrt', [rms_acc2 '/Sqrt'], 'Position', [295, 25, 335, 60]);
    add_line(rms_acc2, 'acc_in/1', 'Square/1');
    add_line(rms_acc2, 'Square/1', 'Integrator/1');
    add_line(rms_acc2, 'Integrator/1', 'Gain_1_T/1');
    add_line(rms_acc2, 'Gain_1_T/1', 'Sqrt/1');
    add_line(rms_acc2, 'Sqrt/1', 'rms_acc/1');

    add_block('simulink/Sinks/Display', [model_name '/Display_RMS_Accel_LQR_ms2'], ...
        'Position', [1285, 560, 1375, 590]);
    add_line(model_name, 'Open-loop Simulation1/2', 'RMS_Accel_LQR/1', 'autorouting', 'on');
    add_line(model_name, 'RMS_Accel_LQR/1', 'Display_RMS_Accel_LQR_ms2/1');
end

rms_acc3 = [model_name '/RMS_Accel_STSMC'];
if getSimulinkBlockHandle(rms_acc3) == -1
    add_block('simulink/Ports & Subsystems/Subsystem', rms_acc3, 'Position', [1205, 645, 1235, 675]);
    delete_line(rms_acc3, 'In1/1', 'Out1/1');
    set_param([rms_acc3 '/In1'], 'Name', 'acc_in', 'Position', [20, 35, 50, 50]);
    set_param([rms_acc3 '/Out1'], 'Name', 'rms_acc', 'Position', [380, 35, 410, 50]);
    add_block('simulink/Math Operations/Math Function', [rms_acc3 '/Square'], 'Operator', 'square', 'Position', [80, 25, 110, 60]);
    add_block('simulink/Continuous/Integrator', [rms_acc3 '/Integrator'], ...
        'LimitOutput', 'on', 'LowerSaturationLimit', '0', 'Position', [145, 25, 175, 60]);
    add_block('simulink/Math Operations/Gain', [rms_acc3 '/Gain_1_T'], 'Gain', '1/10', 'Position', [215, 25, 255, 60]);
    add_block('simulink/Math Operations/Sqrt', [rms_acc3 '/Sqrt'], 'Position', [295, 25, 335, 60]);
    add_line(rms_acc3, 'acc_in/1', 'Square/1');
    add_line(rms_acc3, 'Square/1', 'Integrator/1');
    add_line(rms_acc3, 'Integrator/1', 'Gain_1_T/1');
    add_line(rms_acc3, 'Gain_1_T/1', 'Sqrt/1');
    add_line(rms_acc3, 'Sqrt/1', 'rms_acc/1');

    add_block('simulink/Sinks/Display', [model_name '/Display_RMS_Accel_STSMC_ms2'], ...
        'Position', [1285, 645, 1375, 675]);
    add_line(model_name, 'Open-loop Simulation2/2', 'RMS_Accel_STSMC/1', 'autorouting', 'on');
    add_line(model_name, 'RMS_Accel_STSMC/1', 'Display_RMS_Accel_STSMC_ms2/1');
end

% Set professional labels on comparison scopes using SIDs
try
    set_param(Simulink.ID.getHandle([model_name ':139']), 'Name', 'Comparison_Body_Acceleration_zs_ddot');
    set_param(Simulink.ID.getHandle([model_name ':140']), 'Name', 'Comparison_Suspension_Deflection');
    set_param(Simulink.ID.getHandle([model_name ':141']), 'Name', 'Comparison_Actuator_Force_Fc');

    set_param(Simulink.ID.getHandle([model_name ':1']),  'Name', 'Simulation_Passive_zr_zs_zus');
    set_param(Simulink.ID.getHandle([model_name ':2']),  'Name', 'Simulation_LQR_zr_zs_zus');
    set_param(Simulink.ID.getHandle([model_name ':64']), 'Name', 'Simulation_STSMC_zr_zs_zus');
catch ME
    fprintf('Warning updating scope names: %s\n', ME.message);
end

fprintf('[7/7] Adding Base Workspace Loggers for Automated Analysis...\n');

% To Workspace Loggers
loggers = { ...
    'Differentiator/1',         'zr_dot_sim',          [200, 15, 260, 35]; ...
    'Open-loop Simulation/2',  'zs_ddot_passive_sim',  [600, 100, 710, 120]; ...
    'Open-loop Simulation1/2', 'zs_ddot_lqr_sim',      [600, 335, 710, 355]; ...
    'Open-loop Simulation2/2', 'zs_ddot_stsmc_sim',    [600, 585, 710, 605]; ...
    'Open-loop Simulation/3',  'defl_passive_sim',     [600, 135, 710, 155]; ...
    'Open-loop Simulation1/3', 'defl_lqr_sim',         [600, 370, 710, 390]; ...
    'Open-loop Simulation2/3', 'defl_stsmc_sim',       [600, 620, 710, 640]; ...
    'Saturation/1',            'Fc_passive_sim',       [385, 80, 475, 100]; ...
    'Saturation1/1',           'Fc_lqr_sim',           [360, 325, 450, 345]; ...
    'Saturation2/1',           'Fc_stsmc_sim',         [340, 575, 430, 595]; ...
    'Road Surface Generator/1', 'zr_sim',              [100, 15, 160, 35]; ...
    'Open-loop Simulation2/3',  'susp_defl_sim',       [750, 620, 850, 640]; ...
    'Open-loop Simulation2/2',  'zs_ddot_sim',         [750, 585, 850, 605]; ...
    'Saturation2/1',            'Fc_force_sim',        [470, 575, 560, 595]  ...
};

for k = 1:size(loggers, 1)
    src_p = loggers{k, 1};
    vname = loggers{k, 2};
    pos_v = loggers{k, 3};
    lblk  = [model_name '/' vname];
    if getSimulinkBlockHandle(lblk) == -1
        add_block('simulink/Sinks/To Workspace', lblk, ...
            'VariableName', vname, 'SaveFormat', 'Array', 'Position', pos_v);
        add_line(model_name, src_p, [vname '/1'], 'autorouting', 'on');
    end
end


save_system(model_name, fullfile(pwd, [model_name '.slx']));
close_system(model_name, 0);

fprintf('\n========================================================================\n');
fprintf('  UNIFIED COMPARATIVE SIMULINK MODEL %s.slx BUILT SUCCESSFULLY!\n', model_name);
fprintf('  Format: Exact Dinesh CV Results Structure\n');
fprintf('  Branch 1: Passive Baseline (with Manual Switch to Classical LQR)\n');
fprintf('  Branch 2: Optimal LQR (ACO-Tuned Feedback)\n');
fprintf('  Branch 3: Adaptive STSMC (Continuous 2nd-Order Super-Twisting)\n');
fprintf('  Includes: 6 Road Profiles, Continuous RMS Subsystems, Displays, and Scopes\n');
fprintf('========================================================================\n\n');
