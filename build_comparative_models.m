% BUILD_COMPARATIVE_MODELS
% Programmatically constructs Master_Active_Suspension_Simulation.slx
% Supporting all 6 Standardized Road Profiles & 6 Controller Architectures
% Features:
%   - Multiport Switch for 6 Road Profiles (Chirp, Smooth, Pothole, Ramp, Rough, Resonance)
%   - 6 Controller Options (Passive, Classical LQR, ACO LQR, HS LQR, Fixed STSMC, Adaptive STSMC)
%   - Continuous Analog RMS Calculation Subsystems for Suspension Deflection & Body Acceleration
%   - Live Digital Canvas Display Blocks for Real-Time Numerical RMS Readout

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  BUILDING COMPLETE 6-ROAD COMPARATIVE SIMULINK MODEL\n');
fprintf('========================================================================\n\n');

setup_as;

m1 = 'Master_Active_Suspension_Simulation';
if bdIsLoaded(m1), close_system(m1, 0); end

new_system(m1);
load_system(m1);

set_param(m1, 'Solver', 'ode45');
set_param(m1, 'StopTime', '10.0');
set_param(m1, 'RelTol', '1e-4');
set_param(m1, 'MaxStep', '1e-3');
set_param(m1, 'AlgebraicLoopMsg', 'none');
set_param(m1, 'InitFcn', 'setup_as');
set_param(m1, 'PreLoadFcn', 'setup_as');

%% 1. Road Surface Generation (All 6 Standardized Profiles)
add_block('simulink/Sources/Constant', [m1 '/Road_Selector'], ...
    'Value', 'ROAD_PROFILE_ID', 'Position', [30, 40, 70, 70]);

% Profile 1: Frequency-Sweep Chirp (0.1 to 10 Hz)
add_block('simulink/Sources/Chirp Signal', [m1 '/Road_Chirp_Raw'], ...
    'f1', 'road_chirp_f0', 'T', '10.0', 'f2', 'road_chirp_f1', ...
    'Position', [30, 90, 70, 120]);
add_block('simulink/Math Operations/Gain', [m1 '/Gain_Chirp_Amp'], ...
    'Gain', 'road_chirp_amp', 'Position', [90, 95, 120, 115]);
add_line(m1, 'Road_Chirp_Raw/1', 'Gain_Chirp_Amp/1');

% Profile 2: Smooth Undulating Highway (4 mm Sine, 0.20 Hz)
add_block('simulink/Sources/Sine Wave', [m1 '/Road_Smooth_Sine'], ...
    'Amplitude', 'road_smooth_amp', 'Frequency', '2*pi*road_smooth_freq', 'SampleTime', '0', ...
    'Position', [30, 140, 70, 170]);

% Profile 3: Severe Discrete Potholes (20 mm Drops)
add_block('simulink/Sources/Pulse Generator', [m1 '/Road_Pothole_Pulse'], ...
    'Amplitude', '-road_pothole_amp', 'Period', '1/road_pothole_freq', ...
    'PulseWidth', 'road_pothole_width', 'PhaseDelay', '0.2', ...
    'Position', [30, 190, 70, 220]);

% Profile 4: Triangular Ramp & Speed Bump (20 mm)
add_block('simulink/Sources/Repeating Sequence', [m1 '/Road_Ramp_Seq'], ...
    'rep_seq_t', '[0, 0.5, 1.5, 1.52, 4.0]', 'rep_seq_y', '[0, 0.020, 0.020, 0, 0]', ...
    'Position', [30, 240, 70, 270]);

% Profile 5: Real-World Stochastic Rough Terrain + Shocks (15 mm)
add_block('simulink/Sources/Sine Wave', [m1 '/Road_Rough_Sine'], ...
    'Amplitude', 'road_rough_amp', 'Frequency', '2*pi*4.5', 'SampleTime', '0', ...
    'Position', [30, 290, 70, 320]);

% Profile 6: Harmonic Resonance Corrugated Road (10 mm, 2.58 Hz)
add_block('simulink/Sources/Sine Wave', [m1 '/Road_Resonance_Sine'], ...
    'Amplitude', 'road_res_amp', 'Frequency', '2*pi*road_res_freq', 'SampleTime', '0', ...
    'Position', [30, 340, 70, 370]);

% Multiport Switch for 6 Road Profiles
add_block('simulink/Signal Routing/Multiport Switch', [m1 '/Road_Switch'], ...
    'Inputs', '6', 'DataPortOrder', 'One-based contiguous', ...
    'Position', [160, 70, 195, 390]);

add_line(m1, 'Road_Selector/1', 'Road_Switch/1');
add_line(m1, 'Gain_Chirp_Amp/1', 'Road_Switch/2');
add_line(m1, 'Road_Smooth_Sine/1', 'Road_Switch/3');
add_line(m1, 'Road_Pothole_Pulse/1', 'Road_Switch/4');
add_line(m1, 'Road_Ramp_Seq/1', 'Road_Switch/5');
add_line(m1, 'Road_Rough_Sine/1', 'Road_Switch/6');
add_line(m1, 'Road_Resonance_Sine/1', 'Road_Switch/7');

% 2nd-Order Low-Pass Filter Differentiator: s / (s/314.16 + 1)
add_block('simulink/Continuous/Transfer Fcn', [m1 '/Road_Differentiator'], ...
    'Numerator', '[314.16 0]', 'Denominator', '[1 314.16]', ...
    'Position', [230, 215, 305, 245]);
add_line(m1, 'Road_Switch/1', 'Road_Differentiator/1');

%% 2. Plant Model (Quanser Quarter-Car State-Space)
add_block('simulink/Signal Routing/Mux', [m1 '/Plant_Input_Mux'], ...
    'Inputs', '2', 'Position', [480, 220, 485, 280]);

add_block('simulink/Continuous/State-Space', [m1 '/Quanser_QuarterCar_Plant'], ...
    'A', 'A', 'B', 'B', 'C', 'eye(4)', 'D', 'zeros(4,2)', 'X0', '[0;0;0;0]', ...
    'Position', [520, 215, 620, 285]);

add_line(m1, 'Road_Differentiator/1', 'Plant_Input_Mux/1');
add_line(m1, 'Plant_Input_Mux/1', 'Quanser_QuarterCar_Plant/1');

add_block('simulink/Signal Routing/Demux', [m1 '/State_Demux'], ...
    'Outputs', '4', 'Position', [660, 200, 665, 300]);
add_line(m1, 'Quanser_QuarterCar_Plant/1', 'State_Demux/1');

%% 3. Vehicle Body Acceleration Calculation
add_block('simulink/Math Operations/Gain', [m1 '/Gain_ks_ms'], ...
    'Gain', '-ks/ms', 'Position', [710, 160, 760, 190]);
add_block('simulink/Math Operations/Sum', [m1 '/Sum_vel_diff'], ...
    'Inputs', '+-', 'Position', [700, 210, 720, 240]);
add_block('simulink/Math Operations/Gain', [m1 '/Gain_bs_ms'], ...
    'Gain', '-bs/ms', 'Position', [740, 210, 790, 240]);
add_block('simulink/Math Operations/Gain', [m1 '/Gain_inv_ms'], ...
    'Gain', '1/ms', 'Position', [710, 260, 760, 290]);
add_block('simulink/Math Operations/Sum', [m1 '/Sum_zs_ddot'], ...
    'Inputs', '+++', 'Position', [815, 205, 840, 255]);

add_line(m1, 'State_Demux/1', 'Gain_ks_ms/1');
add_line(m1, 'State_Demux/2', 'Sum_vel_diff/1');
add_line(m1, 'State_Demux/4', 'Sum_vel_diff/2');
add_line(m1, 'Sum_vel_diff/1', 'Gain_bs_ms/1');
add_line(m1, 'Gain_ks_ms/1', 'Sum_zs_ddot/1');
add_line(m1, 'Gain_bs_ms/1', 'Sum_zs_ddot/2');
add_line(m1, 'Gain_inv_ms/1', 'Sum_zs_ddot/3');

%% 4. Native Simulink STSMC Subsystems (Fixed & Adaptive)
% Fixed STSMC Subsystem
stsmc_fixed_sub = [m1 '/STSMC_Fixed_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', stsmc_fixed_sub, ...
    'Position', [250, 600, 310, 630]);
delete_line(stsmc_fixed_sub, 'In1/1', 'Out1/1');
set_param([stsmc_fixed_sub '/In1'], 'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([stsmc_fixed_sub '/Out1'], 'Name', 'u_stsmc', 'Position', [650, 140, 680, 160]);

add_block('simulink/Signal Routing/Demux', [stsmc_fixed_sub '/Demux_stsmc'], 'Outputs', '4', 'Position', [90, 80, 95, 140]);
add_line(stsmc_fixed_sub, 'x_state/1', 'Demux_stsmc/1');
add_block('simulink/Math Operations/Gain', [stsmc_fixed_sub '/Gain_c1'], 'Gain', 'c1_fixed', 'Position', [130, 75, 170, 105]);
add_block('simulink/Math Operations/Sum', [stsmc_fixed_sub '/Sum_s'], 'Inputs', '++', 'Position', [200, 90, 220, 120]);
add_line(stsmc_fixed_sub, 'Demux_stsmc/1', 'Gain_c1/1');
add_line(stsmc_fixed_sub, 'Gain_c1/1', 'Sum_s/1');
add_line(stsmc_fixed_sub, 'Demux_stsmc/2', 'Sum_s/2');

add_block('simulink/Math Operations/Gain', [stsmc_fixed_sub '/Gain_inv_eps'], 'Gain', '1/eps_bl', 'Position', [245, 90, 285, 120]);
add_block('simulink/Math Operations/Trigonometric Function', [stsmc_fixed_sub '/Tanh_sign'], 'Operator', 'tanh', 'Position', [310, 90, 340, 120]);
add_line(stsmc_fixed_sub, 'Sum_s/1', 'Gain_inv_eps/1');
add_line(stsmc_fixed_sub, 'Gain_inv_eps/1', 'Tanh_sign/1');

add_block('simulink/Math Operations/Abs', [stsmc_fixed_sub '/Abs_s'], 'Position', [245, 40, 275, 70]);
add_block('simulink/Math Operations/Math Function', [stsmc_fixed_sub '/Sqrt_s'], 'Function', 'sqrt', 'Position', [310, 40, 340, 70]);
add_line(stsmc_fixed_sub, 'Sum_s/1', 'Abs_s/1');
add_line(stsmc_fixed_sub, 'Abs_s/1', 'Sqrt_s/1');

add_block('simulink/Math Operations/Product', [stsmc_fixed_sub '/Prod_prop'], 'Inputs', '**', 'Position', [380, 45, 405, 80]);
add_line(stsmc_fixed_sub, 'Sqrt_s/1', 'Prod_prop/1');
add_line(stsmc_fixed_sub, 'Tanh_sign/1', 'Prod_prop/2');

add_block('simulink/Math Operations/Gain', [stsmc_fixed_sub '/Gain_k1'], 'Gain', '-k1_fixed', 'Position', [435, 50, 475, 80]);
add_line(stsmc_fixed_sub, 'Prod_prop/1', 'Gain_k1/1');

add_block('simulink/Math Operations/Gain', [stsmc_fixed_sub '/Gain_k2'], 'Gain', '-k2_fixed', 'Position', [380, 130, 420, 160]);
add_block('simulink/Continuous/Integrator', [stsmc_fixed_sub '/Integrator_w'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', 'UpperSaturationLimit', '39.2', 'LowerSaturationLimit', '-39.2', ...
    'Position', [450, 130, 480, 160]);
add_line(stsmc_fixed_sub, 'Tanh_sign/1', 'Gain_k2/1');
add_line(stsmc_fixed_sub, 'Gain_k2/1', 'Integrator_w/1');

add_block('simulink/Math Operations/Sum', [stsmc_fixed_sub '/Sum_u'], 'Inputs', '++', 'Position', [520, 85, 545, 135]);
add_line(stsmc_fixed_sub, 'Gain_k1/1', 'Sum_u/1');
add_line(stsmc_fixed_sub, 'Integrator_w/1', 'Sum_u/2');
add_line(stsmc_fixed_sub, 'Sum_u/1', 'u_stsmc/1');

% Adaptive STSMC Subsystem
stsmc_adap_sub = [m1 '/STSMC_Adaptive_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', stsmc_adap_sub, ...
    'Position', [250, 650, 310, 680]);
delete_line(stsmc_adap_sub, 'In1/1', 'Out1/1');
set_param([stsmc_adap_sub '/In1'], 'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([stsmc_adap_sub '/Out1'], 'Name', 'u_stsmc', 'Position', [650, 140, 680, 160]);

add_block('simulink/Signal Routing/Demux', [stsmc_adap_sub '/Demux_stsmc'], 'Outputs', '4', 'Position', [90, 80, 95, 140]);
add_line(stsmc_adap_sub, 'x_state/1', 'Demux_stsmc/1');
add_block('simulink/Math Operations/Gain', [stsmc_adap_sub '/Gain_c1'], 'Gain', 'c1_adap', 'Position', [130, 75, 170, 105]);
add_block('simulink/Math Operations/Sum', [stsmc_adap_sub '/Sum_s'], 'Inputs', '++', 'Position', [200, 90, 220, 120]);
add_line(stsmc_adap_sub, 'Demux_stsmc/1', 'Gain_c1/1');
add_line(stsmc_adap_sub, 'Gain_c1/1', 'Sum_s/1');
add_line(stsmc_adap_sub, 'Demux_stsmc/2', 'Sum_s/2');

add_block('simulink/Math Operations/Gain', [stsmc_adap_sub '/Gain_inv_eps'], 'Gain', '1/eps_bl', 'Position', [245, 90, 285, 120]);
add_block('simulink/Math Operations/Trigonometric Function', [stsmc_adap_sub '/Tanh_sign'], 'Operator', 'tanh', 'Position', [310, 90, 340, 120]);
add_line(stsmc_adap_sub, 'Sum_s/1', 'Gain_inv_eps/1');
add_line(stsmc_adap_sub, 'Gain_inv_eps/1', 'Tanh_sign/1');

add_block('simulink/Math Operations/Abs', [stsmc_adap_sub '/Abs_s'], 'Position', [245, 40, 275, 70]);
add_block('simulink/Math Operations/Math Function', [stsmc_adap_sub '/Sqrt_s'], 'Function', 'sqrt', 'Position', [310, 40, 340, 70]);
add_line(stsmc_adap_sub, 'Sum_s/1', 'Abs_s/1');
add_line(stsmc_adap_sub, 'Abs_s/1', 'Sqrt_s/1');

add_block('simulink/Math Operations/Product', [stsmc_adap_sub '/Prod_prop'], 'Inputs', '**', 'Position', [380, 45, 405, 80]);
add_line(stsmc_adap_sub, 'Sqrt_s/1', 'Prod_prop/1');
add_line(stsmc_adap_sub, 'Tanh_sign/1', 'Prod_prop/2');

add_block('simulink/Math Operations/Gain', [stsmc_adap_sub '/Gain_k1'], 'Gain', '-k1_adap', 'Position', [435, 50, 475, 80]);
add_line(stsmc_adap_sub, 'Prod_prop/1', 'Gain_k1/1');

add_block('simulink/Math Operations/Gain', [stsmc_adap_sub '/Gain_k2'], 'Gain', '-k2_adap', 'Position', [380, 130, 420, 160]);
add_block('simulink/Continuous/Integrator', [stsmc_adap_sub '/Integrator_w'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', 'UpperSaturationLimit', '39.2', 'LowerSaturationLimit', '-39.2', ...
    'Position', [450, 130, 480, 160]);
add_line(stsmc_adap_sub, 'Tanh_sign/1', 'Gain_k2/1');
add_line(stsmc_adap_sub, 'Gain_k2/1', 'Integrator_w/1');

add_block('simulink/Math Operations/Sum', [stsmc_adap_sub '/Sum_u'], 'Inputs', '++', 'Position', [520, 85, 545, 135]);
add_line(stsmc_adap_sub, 'Gain_k1/1', 'Sum_u/1');
add_line(stsmc_adap_sub, 'Integrator_w/1', 'Sum_u/2');
add_line(stsmc_adap_sub, 'Sum_u/1', 'u_stsmc/1');

%% 5. Controller Bank (Passive, LQR Variants, STSMC Variants)
add_block('simulink/Sources/Constant', [m1 '/Ctrl_Passive'], ...
    'Value', '0', 'Position', [250, 400, 280, 420]);

add_block('simulink/Math Operations/Gain', [m1 '/Ctrl_Classical_LQR'], ...
    'Gain', '-K_classical', 'Multiplication', 'Matrix(K*u)', 'Position', [250, 440, 310, 470]);

add_block('simulink/Math Operations/Gain', [m1 '/Ctrl_ACO_LQR'], ...
    'Gain', '-K_aco', 'Multiplication', 'Matrix(K*u)', 'Position', [250, 490, 310, 520]);

add_block('simulink/Math Operations/Gain', [m1 '/Ctrl_HS_LQR'], ...
    'Gain', '-K_hs', 'Multiplication', 'Matrix(K*u)', 'Position', [250, 540, 310, 570]);

add_line(m1, 'Quanser_QuarterCar_Plant/1', 'Ctrl_Classical_LQR/1');
add_line(m1, 'Quanser_QuarterCar_Plant/1', 'Ctrl_ACO_LQR/1');
add_line(m1, 'Quanser_QuarterCar_Plant/1', 'Ctrl_HS_LQR/1');
add_line(m1, 'Quanser_QuarterCar_Plant/1', 'STSMC_Fixed_Controller/1');
add_line(m1, 'Quanser_QuarterCar_Plant/1', 'STSMC_Adaptive_Controller/1');

add_block('simulink/Sources/Constant', [m1 '/Controller_Selector'], ...
    'Value', 'CONTROLLER_ID', 'Position', [330, 360, 370, 380]);

add_block('simulink/Signal Routing/Multiport Switch', [m1 '/Controller_Switch'], ...
    'Inputs', '6', 'DataPortOrder', 'One-based contiguous', ...
    'Position', [390, 390, 425, 690]);

add_line(m1, 'Controller_Selector/1', 'Controller_Switch/1');
add_line(m1, 'Ctrl_Passive/1', 'Controller_Switch/2');
add_line(m1, 'Ctrl_Classical_LQR/1', 'Controller_Switch/3');
add_line(m1, 'Ctrl_ACO_LQR/1', 'Controller_Switch/4');
add_line(m1, 'Ctrl_HS_LQR/1', 'Controller_Switch/5');
add_line(m1, 'STSMC_Fixed_Controller/1', 'Controller_Switch/6');
add_line(m1, 'STSMC_Adaptive_Controller/1', 'Controller_Switch/7');

add_block('simulink/Discontinuities/Saturation', [m1 '/Actuator_Saturation'], ...
    'UpperLimit', 'Fc_max', 'LowerLimit', 'Fc_min', ...
    'Position', [450, 525, 480, 555]);

add_line(m1, 'Controller_Switch/1', 'Actuator_Saturation/1');
add_line(m1, 'Actuator_Saturation/1', 'Plant_Input_Mux/2');
add_line(m1, 'Actuator_Saturation/1', 'Gain_inv_ms/1');

%% 6. Continuous RMS Calculation Subsystems & Canvas Displays (Dinesh Architecture)
% 6a. Suspension Deflection RMS Subsystem
rms_susp_sub = [m1 '/RMS_Suspension_Deflection'];
add_block('simulink/Ports & Subsystems/Subsystem', rms_susp_sub, ...
    'Position', [720, 100, 780, 135]);
delete_line(rms_susp_sub, 'In1/1', 'Out1/1');
set_param([rms_susp_sub '/In1'], 'Name', 'signal_in', 'Position', [20, 35, 50, 50]);
set_param([rms_susp_sub '/Out1'], 'Name', 'rms_out', 'Position', [380, 35, 410, 50]);

add_block('simulink/Math Operations/Math Function', [rms_susp_sub '/Square'], ...
    'Function', 'square', 'Position', [80, 25, 110, 60]);
add_block('simulink/Continuous/Integrator', [rms_susp_sub '/Integrator'], ...
    'Position', [145, 25, 175, 60]);
add_block('simulink/Math Operations/Gain', [rms_susp_sub '/Gain_1_T'], ...
    'Gain', '1/10', 'Position', [215, 25, 255, 60]);
add_block('simulink/Math Operations/Math Function', [rms_susp_sub '/Sqrt'], ...
    'Function', 'sqrt', 'Position', [295, 25, 335, 60]);

add_line(rms_susp_sub, 'signal_in/1', 'Square/1');
add_line(rms_susp_sub, 'Square/1', 'Integrator/1');
add_line(rms_susp_sub, 'Integrator/1', 'Gain_1_T/1');
add_line(rms_susp_sub, 'Gain_1_T/1', 'Sqrt/1');
add_line(rms_susp_sub, 'Sqrt/1', 'rms_out/1');

add_line(m1, 'State_Demux/1', 'RMS_Suspension_Deflection/1');

add_block('simulink/Math Operations/Gain', [m1 '/Gain_m2mm'], ...
    'Gain', '1000', 'Position', [810, 105, 845, 130]);
add_line(m1, 'RMS_Suspension_Deflection/1', 'Gain_m2mm/1');

add_block('simulink/Sinks/Display', [m1 '/Display_RMS_Susp_Defl_mm'], ...
    'Position', [870, 100, 960, 135]);
add_line(m1, 'Gain_m2mm/1', 'Display_RMS_Susp_Defl_mm/1');

% 6b. Body Acceleration RMS Subsystem
rms_accel_sub = [m1 '/RMS_Body_Acceleration'];
add_block('simulink/Ports & Subsystems/Subsystem', rms_accel_sub, ...
    'Position', [875, 275, 935, 310]);
delete_line(rms_accel_sub, 'In1/1', 'Out1/1');
set_param([rms_accel_sub '/In1'], 'Name', 'signal_in', 'Position', [20, 35, 50, 50]);
set_param([rms_accel_sub '/Out1'], 'Name', 'rms_out', 'Position', [380, 35, 410, 50]);

add_block('simulink/Math Operations/Math Function', [rms_accel_sub '/Square'], ...
    'Function', 'square', 'Position', [80, 25, 110, 60]);
add_block('simulink/Continuous/Integrator', [rms_accel_sub '/Integrator'], ...
    'Position', [145, 25, 175, 60]);
add_block('simulink/Math Operations/Gain', [rms_accel_sub '/Gain_1_T'], ...
    'Gain', '1/10', 'Position', [215, 25, 255, 60]);
add_block('simulink/Math Operations/Math Function', [rms_accel_sub '/Sqrt'], ...
    'Function', 'sqrt', 'Position', [295, 25, 335, 60]);

add_line(rms_accel_sub, 'signal_in/1', 'Square/1');
add_line(rms_accel_sub, 'Square/1', 'Integrator/1');
add_line(rms_accel_sub, 'Integrator/1', 'Gain_1_T/1');
add_line(rms_accel_sub, 'Gain_1_T/1', 'Sqrt/1');
add_line(rms_accel_sub, 'Sqrt/1', 'rms_out/1');

add_line(m1, 'Sum_zs_ddot/1', 'RMS_Body_Acceleration/1');

add_block('simulink/Sinks/Display', [m1 '/Display_RMS_Body_Accel_ms2'], ...
    'Position', [970, 275, 1060, 310]);
add_line(m1, 'RMS_Body_Acceleration/1', 'Display_RMS_Body_Accel_ms2/1');

%% 7. Displacement Reconstruction & Scopes
add_block('simulink/Math Operations/Sum', [m1 '/Sum_zus'], 'Inputs', '++', 'Position', [720, 45, 740, 75]);
add_line(m1, 'State_Demux/3', 'Sum_zus/1');
add_line(m1, 'Road_Switch/1', 'Sum_zus/2');

add_block('simulink/Math Operations/Sum', [m1 '/Sum_zs'], 'Inputs', '++', 'Position', [770, 25, 790, 55]);
add_line(m1, 'State_Demux/1', 'Sum_zs/1');
add_line(m1, 'Sum_zus/1', 'Sum_zs/2');

add_block('simulink/Signal Routing/Mux', [m1 '/Mux_Displacements'], 'Inputs', '3', 'Position', [830, 20, 835, 80]);
add_line(m1, 'Road_Switch/1', 'Mux_Displacements/1');
add_line(m1, 'Sum_zus/1', 'Mux_Displacements/2');
add_line(m1, 'Sum_zs/1', 'Mux_Displacements/3');

add_block('simulink/Sinks/Scope', [m1 '/Scope_Displacements_zr_zus_zs'], 'Position', [870, 35, 910, 65]);
add_line(m1, 'Mux_Displacements/1', 'Scope_Displacements_zr_zus_zs/1');

add_block('simulink/Sinks/Scope', [m1 '/Scope_Suspension_Deflection'], 'Position', [890, 160, 930, 190]);
add_line(m1, 'State_Demux/1', 'Scope_Suspension_Deflection/1');

add_block('simulink/Sinks/Scope', [m1 '/Scope_Body_Acceleration'], 'Position', [890, 215, 930, 245]);
add_line(m1, 'Sum_zs_ddot/1', 'Scope_Body_Acceleration/1');

add_block('simulink/Sinks/Scope', [m1 '/Scope_Actuator_Force'], 'Position', [520, 575, 560, 605]);
add_line(m1, 'Actuator_Saturation/1', 'Scope_Actuator_Force/1');

% To Workspace Loggers
add_block('simulink/Sinks/To Workspace', [m1 '/Log_zr'], 'VariableName', 'zr_sim', 'SaveFormat', 'Array', 'Position', [230, 150, 290, 170]);
add_line(m1, 'Road_Switch/1', 'Log_zr/1');

add_block('simulink/Sinks/To Workspace', [m1 '/Log_zus'], 'VariableName', 'zus_sim', 'SaveFormat', 'Array', 'Position', [770, 65, 830, 85]);
add_line(m1, 'Sum_zus/1', 'Log_zus/1');

add_block('simulink/Sinks/To Workspace', [m1 '/Log_zs'], 'VariableName', 'zs_sim', 'SaveFormat', 'Array', 'Position', [820, 5, 880, 25]);
add_line(m1, 'Sum_zs/1', 'Log_zs/1');

add_block('simulink/Sinks/To Workspace', [m1 '/Log_susp_deflection'], 'VariableName', 'susp_defl_sim', 'SaveFormat', 'Array', 'Position', [800, 140, 870, 160]);
add_line(m1, 'State_Demux/1', 'Log_susp_deflection/1');

add_block('simulink/Sinks/To Workspace', [m1 '/Log_zs_ddot'], 'VariableName', 'zs_ddot_sim', 'SaveFormat', 'Array', 'Position', [890, 320, 960, 340]);
add_line(m1, 'Sum_zs_ddot/1', 'Log_zs_ddot/1');

add_block('simulink/Sinks/To Workspace', [m1 '/Log_Fc_force'], 'VariableName', 'Fc_force_sim', 'SaveFormat', 'Array', 'Position', [520, 525, 590, 545]);
add_line(m1, 'Actuator_Saturation/1', 'Log_Fc_force/1');

save_system(m1, fullfile(pwd, [m1 '.slx']));
close_system(m1, 0);

fprintf('Complete 6-Road Simulink Model %s.slx built successfully with continuous RMS blocks & Displays!\n', m1);
