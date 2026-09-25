% BUILD_NATIVE_SIMULINK_MODEL
% Constructs the Pure Native Simulink Model for Quanser Active Suspension with STSMC

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  BUILDING PURE NATIVE SIMULINK MODEL (ZERO COMPILER/DISCREPANCY ISSUES)\n');
fprintf('========================================================================\n\n');

setup_as;

model_name = 'Master_Active_Suspension_Simulation';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

new_system(model_name);
load_system(model_name);

set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'StopTime', '10.0');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '1e-3');
set_param(model_name, 'AlgebraicLoopMsg', 'none');
set_param(model_name, 'InitFcn', 'setup_as');
set_param(model_name, 'PreLoadFcn', 'setup_as');
set_param(model_name, 'StopFcn', 'auto_plot_results');

%% 1. Road Surface Generation
add_block('simulink/Sources/Constant', [model_name '/Road_Selector'], ...
    'Value', 'ROAD_PROFILE_ID', 'Position', [30, 100, 70, 130]);

add_block('simulink/Sources/Sine Wave', [model_name '/Road_Smooth_Sine'], ...
    'Amplitude', 'road_smooth_amp', 'Frequency', '2*pi*road_smooth_freq', 'SampleTime', '0', ...
    'Position', [30, 160, 80, 190]);

add_block('simulink/Sources/Sine Wave', [model_name '/Road_Normal_Sine'], ...
    'Amplitude', 'road_normal_amp', 'Frequency', '2*pi*road_normal_freq', 'SampleTime', '0', ...
    'Position', [30, 210, 80, 240]);

add_block('simulink/Sources/Sine Wave', [model_name '/Road_Rough_Sine'], ...
    'Amplitude', 'road_rough_amp', 'Frequency', '2*pi*2.5', 'SampleTime', '0', ...
    'Position', [30, 260, 80, 290]);

add_block('simulink/Sources/Pulse Generator', [model_name '/Road_Pothole_Pulse'], ...
    'Amplitude', 'road_pothole_amp', 'Period', '1/road_pothole_freq', ...
    'PulseWidth', 'road_pothole_width', 'PhaseDelay', '0', ...
    'Position', [30, 310, 80, 340]);

add_block('simulink/Signal Routing/Multiport Switch', [model_name '/Road_Switch'], ...
    'Inputs', '4', 'DataPortOrder', 'One-based contiguous', ...
    'Position', [140, 150, 180, 350]);

add_line(model_name, 'Road_Selector/1', 'Road_Switch/1');
add_line(model_name, 'Road_Smooth_Sine/1', 'Road_Switch/2');
add_line(model_name, 'Road_Normal_Sine/1', 'Road_Switch/3');
add_line(model_name, 'Road_Rough_Sine/1', 'Road_Switch/4');
add_line(model_name, 'Road_Pothole_Pulse/1', 'Road_Switch/5');

add_block('simulink/Continuous/Transfer Fcn', [model_name '/Road_Differentiator'], ...
    'Numerator', '[314.16 0]', 'Denominator', '[1 314.16]', ...
    'Position', [220, 235, 300, 265]);

add_line(model_name, 'Road_Switch/1', 'Road_Differentiator/1');

%% 2. Plant Model
add_block('simulink/Signal Routing/Mux', [model_name '/Plant_Input_Mux'], ...
    'Inputs', '2', 'Position', [480, 240, 485, 300]);

add_block('simulink/Continuous/State-Space', [model_name '/Quanser_QuarterCar_Plant'], ...
    'A', 'A', 'B', 'B', 'C', 'eye(4)', 'D', 'zeros(4,2)', 'X0', '[0;0;0;0]', ...
    'Position', [520, 235, 620, 305]);

add_line(model_name, 'Road_Differentiator/1', 'Plant_Input_Mux/1');
add_line(model_name, 'Plant_Input_Mux/1', 'Quanser_QuarterCar_Plant/1');

add_block('simulink/Signal Routing/Demux', [model_name '/State_Demux'], ...
    'Outputs', '4', 'Position', [660, 220, 665, 320]);

add_line(model_name, 'Quanser_QuarterCar_Plant/1', 'State_Demux/1');

%% 3. True Body Acceleration
add_block('simulink/Math Operations/Gain', [model_name '/Gain_ks_ms'], ...
    'Gain', '-ks/ms', 'Position', [720, 180, 770, 210]);
add_block('simulink/Math Operations/Sum', [model_name '/Sum_vel_diff'], ...
    'Inputs', '+-', 'Position', [700, 230, 720, 260]);
add_block('simulink/Math Operations/Gain', [model_name '/Gain_bs_ms'], ...
    'Gain', '-bs/ms', 'Position', [740, 230, 790, 260]);
add_block('simulink/Math Operations/Gain', [model_name '/Gain_inv_ms'], ...
    'Gain', '1/ms', 'Position', [720, 280, 770, 310]);
add_block('simulink/Math Operations/Sum', [model_name '/Sum_zs_ddot'], ...
    'Inputs', '+++', 'Position', [820, 220, 845, 270]);

add_line(model_name, 'State_Demux/1', 'Gain_ks_ms/1');
add_line(model_name, 'State_Demux/2', 'Sum_vel_diff/1');
add_line(model_name, 'State_Demux/4', 'Sum_vel_diff/2');
add_line(model_name, 'Sum_vel_diff/1', 'Gain_bs_ms/1');

add_line(model_name, 'Gain_ks_ms/1', 'Sum_zs_ddot/1');
add_line(model_name, 'Gain_bs_ms/1', 'Sum_zs_ddot/2');
add_line(model_name, 'Gain_inv_ms/1', 'Sum_zs_ddot/3');

%% 4. Native Simulink STSMC Subsystems (Fixed and Adaptive)
% Fixed STSMC Subsystem
stsmc_fixed_sub = [model_name '/STSMC_Fixed_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', stsmc_fixed_sub, ...
    'Position', [250, 610, 310, 640]);

delete_line(stsmc_fixed_sub, 'In1/1', 'Out1/1');
set_param([stsmc_fixed_sub '/In1'], 'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([stsmc_fixed_sub '/Out1'], 'Name', 'u_stsmc', 'Position', [650, 140, 680, 160]);

add_block('simulink/Signal Routing/Demux', [stsmc_fixed_sub '/Demux_stsmc'], ...
    'Outputs', '4', 'Position', [90, 80, 95, 140]);
add_line(stsmc_fixed_sub, 'x_state/1', 'Demux_stsmc/1');

add_block('simulink/Math Operations/Gain', [stsmc_fixed_sub '/Gain_c1'], ...
    'Gain', 'c1_fixed', 'Position', [130, 75, 170, 105]);
add_block('simulink/Math Operations/Sum', [stsmc_fixed_sub '/Sum_s'], ...
    'Inputs', '++', 'Position', [200, 90, 220, 120]);

add_line(stsmc_fixed_sub, 'Demux_stsmc/1', 'Gain_c1/1');
add_line(stsmc_fixed_sub, 'Gain_c1/1', 'Sum_s/1');
add_line(stsmc_fixed_sub, 'Demux_stsmc/2', 'Sum_s/2');

add_block('simulink/Math Operations/Abs', [stsmc_fixed_sub '/Abs_s'], ...
    'Position', [250, 60, 280, 90]);
add_block('simulink/Sources/Constant', [stsmc_fixed_sub '/Const_eps_bl'], ...
    'Value', 'eps_bl', 'Position', [250, 110, 280, 130]);
add_block('simulink/Math Operations/Sum', [stsmc_fixed_sub '/Sum_denom'], ...
    'Inputs', '++', 'Position', [310, 80, 330, 110]);
add_block('simulink/Math Operations/Divide', [stsmc_fixed_sub '/Div_sign_s'], ...
    'Inputs', '*/', 'Position', [360, 75, 385, 115]);

add_line(stsmc_fixed_sub, 'Sum_s/1', 'Abs_s/1');
add_line(stsmc_fixed_sub, 'Abs_s/1', 'Sum_denom/1');
add_line(stsmc_fixed_sub, 'Const_eps_bl/1', 'Sum_denom/2');
add_line(stsmc_fixed_sub, 'Sum_s/1', 'Div_sign_s/1');
add_line(stsmc_fixed_sub, 'Sum_denom/1', 'Div_sign_s/2');

add_block('simulink/Math Operations/Math Function', [stsmc_fixed_sub '/Sqrt_abs_s'], ...
    'Function', 'sqrt', 'Position', [320, 30, 350, 60]);
add_block('simulink/Math Operations/Product', [stsmc_fixed_sub '/Prod_prop'], ...
    'Inputs', '**', 'Position', [410, 45, 435, 80]);
add_block('simulink/Math Operations/Gain', [stsmc_fixed_sub '/Gain_k1'], ...
    'Gain', '-k1_fixed', 'Position', [460, 50, 500, 80]);

add_line(stsmc_fixed_sub, 'Abs_s/1', 'Sqrt_abs_s/1');
add_line(stsmc_fixed_sub, 'Sqrt_abs_s/1', 'Prod_prop/1');
add_line(stsmc_fixed_sub, 'Div_sign_s/1', 'Prod_prop/2');
add_line(stsmc_fixed_sub, 'Prod_prop/1', 'Gain_k1/1');

add_block('simulink/Math Operations/Gain', [stsmc_fixed_sub '/Gain_k2'], ...
    'Gain', '-k2_fixed', 'Position', [420, 130, 460, 160]);
add_block('simulink/Continuous/Integrator', [stsmc_fixed_sub '/Integrator_w_int'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', 'UpperSaturationLimit', '39.2', 'LowerSaturationLimit', '-39.2', ...
    'Position', [490, 130, 520, 160]);

add_line(stsmc_fixed_sub, 'Div_sign_s/1', 'Gain_k2/1');
add_line(stsmc_fixed_sub, 'Gain_k2/1', 'Integrator_w_int/1');

add_block('simulink/Math Operations/Sum', [stsmc_fixed_sub '/Sum_u_total'], ...
    'Inputs', '++', 'Position', [550, 95, 575, 145]);

add_line(stsmc_fixed_sub, 'Gain_k1/1', 'Sum_u_total/1');
add_line(stsmc_fixed_sub, 'Integrator_w_int/1', 'Sum_u_total/2');
add_line(stsmc_fixed_sub, 'Sum_u_total/1', 'u_stsmc/1');

% Adaptive STSMC Subsystem
stsmc_adap_sub = [model_name '/STSMC_Adaptive_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', stsmc_adap_sub, ...
    'Position', [250, 660, 310, 690]);

delete_line(stsmc_adap_sub, 'In1/1', 'Out1/1');
set_param([stsmc_adap_sub '/In1'], 'Name', 'x_state', 'Position', [30, 100, 60, 120]);
set_param([stsmc_adap_sub '/Out1'], 'Name', 'u_stsmc', 'Position', [650, 140, 680, 160]);

add_block('simulink/Signal Routing/Demux', [stsmc_adap_sub '/Demux_stsmc'], ...
    'Outputs', '4', 'Position', [90, 80, 95, 140]);
add_line(stsmc_adap_sub, 'x_state/1', 'Demux_stsmc/1');

add_block('simulink/Math Operations/Gain', [stsmc_adap_sub '/Gain_c1'], ...
    'Gain', 'c1_adap', 'Position', [130, 75, 170, 105]);
add_block('simulink/Math Operations/Sum', [stsmc_adap_sub '/Sum_s'], ...
    'Inputs', '++', 'Position', [200, 90, 220, 120]);

add_line(stsmc_adap_sub, 'Demux_stsmc/1', 'Gain_c1/1');
add_line(stsmc_adap_sub, 'Gain_c1/1', 'Sum_s/1');
add_line(stsmc_adap_sub, 'Demux_stsmc/2', 'Sum_s/2');

add_block('simulink/Math Operations/Abs', [stsmc_adap_sub '/Abs_s'], ...
    'Position', [250, 60, 280, 90]);
add_block('simulink/Sources/Constant', [stsmc_adap_sub '/Const_eps_bl'], ...
    'Value', 'eps_bl', 'Position', [250, 110, 280, 130]);
add_block('simulink/Math Operations/Sum', [stsmc_adap_sub '/Sum_denom'], ...
    'Inputs', '++', 'Position', [310, 80, 330, 110]);
add_block('simulink/Math Operations/Divide', [stsmc_adap_sub '/Div_sign_s'], ...
    'Inputs', '*/', 'Position', [360, 75, 385, 115]);

add_line(stsmc_adap_sub, 'Sum_s/1', 'Abs_s/1');
add_line(stsmc_adap_sub, 'Abs_s/1', 'Sum_denom/1');
add_line(stsmc_adap_sub, 'Const_eps_bl/1', 'Sum_denom/2');
add_line(stsmc_adap_sub, 'Sum_s/1', 'Div_sign_s/1');
add_line(stsmc_adap_sub, 'Sum_denom/1', 'Div_sign_s/2');

add_block('simulink/Math Operations/Math Function', [stsmc_adap_sub '/Sqrt_abs_s'], ...
    'Function', 'sqrt', 'Position', [320, 30, 350, 60]);
add_block('simulink/Math Operations/Product', [stsmc_adap_sub '/Prod_prop'], ...
    'Inputs', '**', 'Position', [410, 45, 435, 80]);
add_block('simulink/Math Operations/Gain', [stsmc_adap_sub '/Gain_k1'], ...
    'Gain', '-k1_adap', 'Position', [460, 50, 500, 80]);

add_line(stsmc_adap_sub, 'Abs_s/1', 'Sqrt_abs_s/1');
add_line(stsmc_adap_sub, 'Sqrt_abs_s/1', 'Prod_prop/1');
add_line(stsmc_adap_sub, 'Div_sign_s/1', 'Prod_prop/2');
add_line(stsmc_adap_sub, 'Prod_prop/1', 'Gain_k1/1');

add_block('simulink/Math Operations/Gain', [stsmc_adap_sub '/Gain_k2'], ...
    'Gain', '-k2_adap', 'Position', [420, 130, 460, 160]);
add_block('simulink/Continuous/Integrator', [stsmc_adap_sub '/Integrator_w_int'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', 'UpperSaturationLimit', '39.2', 'LowerSaturationLimit', '-39.2', ...
    'Position', [490, 130, 520, 160]);

add_line(stsmc_adap_sub, 'Div_sign_s/1', 'Gain_k2/1');
add_line(stsmc_adap_sub, 'Gain_k2/1', 'Integrator_w_int/1');

add_block('simulink/Math Operations/Sum', [stsmc_adap_sub '/Sum_u_total'], ...
    'Inputs', '++', 'Position', [550, 95, 575, 145]);

add_line(stsmc_adap_sub, 'Gain_k1/1', 'Sum_u_total/1');
add_line(stsmc_adap_sub, 'Integrator_w_int/1', 'Sum_u_total/2');
add_line(stsmc_adap_sub, 'Sum_u_total/1', 'u_stsmc/1');

%% 5. Controllers & Switching
add_block('simulink/Sources/Constant', [model_name '/Ctrl_Passive'], ...
    'Value', '0', 'Position', [250, 410, 280, 430]);

add_block('simulink/Math Operations/Gain', [model_name '/Ctrl_Classical_LQR'], ...
    'Gain', '-K_classical', 'Multiplication', 'Matrix(K*u)', 'Position', [250, 450, 310, 480]);

add_block('simulink/Math Operations/Gain', [model_name '/Ctrl_ACO_LQR'], ...
    'Gain', '-K_aco', 'Multiplication', 'Matrix(K*u)', 'Position', [250, 500, 310, 530]);

add_block('simulink/Math Operations/Gain', [model_name '/Ctrl_HS_LQR'], ...
    'Gain', '-K_hs', 'Multiplication', 'Matrix(K*u)', 'Position', [250, 550, 310, 580]);

add_line(model_name, 'Quanser_QuarterCar_Plant/1', 'Ctrl_Classical_LQR/1');
add_line(model_name, 'Quanser_QuarterCar_Plant/1', 'Ctrl_ACO_LQR/1');
add_line(model_name, 'Quanser_QuarterCar_Plant/1', 'Ctrl_HS_LQR/1');
add_line(model_name, 'Quanser_QuarterCar_Plant/1', 'STSMC_Fixed_Controller/1');
add_line(model_name, 'Quanser_QuarterCar_Plant/1', 'STSMC_Adaptive_Controller/1');

add_block('simulink/Sources/Constant', [model_name '/Controller_Selector'], ...
    'Value', 'CONTROLLER_ID', 'Position', [320, 370, 360, 390]);

add_block('simulink/Signal Routing/Multiport Switch', [model_name '/Controller_Switch'], ...
    'Inputs', '6', 'DataPortOrder', 'One-based contiguous', ...
    'Position', [390, 400, 425, 700]);

add_line(model_name, 'Controller_Selector/1', 'Controller_Switch/1');
add_line(model_name, 'Ctrl_Passive/1', 'Controller_Switch/2');
add_line(model_name, 'Ctrl_Classical_LQR/1', 'Controller_Switch/3');
add_line(model_name, 'Ctrl_ACO_LQR/1', 'Controller_Switch/4');
add_line(model_name, 'Ctrl_HS_LQR/1', 'Controller_Switch/5');
add_line(model_name, 'STSMC_Fixed_Controller/1', 'Controller_Switch/6');
add_line(model_name, 'STSMC_Adaptive_Controller/1', 'Controller_Switch/7');

add_block('simulink/Discontinuities/Saturation', [model_name '/Actuator_Saturation'], ...
    'UpperLimit', 'Fc_max', 'LowerLimit', 'Fc_min', ...
    'Position', [450, 535, 480, 565]);

add_line(model_name, 'Controller_Switch/1', 'Actuator_Saturation/1');
add_line(model_name, 'Actuator_Saturation/1', 'Plant_Input_Mux/2');
add_line(model_name, 'Actuator_Saturation/1', 'Gain_inv_ms/1');

%% 6. Displacement Reconstruction (zr, zus, zs)
% Unsprung mass displacement: zus = x3 + zr
add_block('simulink/Math Operations/Sum', [model_name '/Sum_zus'], ...
    'Inputs', '++', 'Position', [720, 70, 740, 100]);
add_line(model_name, 'State_Demux/3', 'Sum_zus/1');
add_line(model_name, 'Road_Switch/1', 'Sum_zus/2');

% Sprung mass displacement: zs = x1 + zus
add_block('simulink/Math Operations/Sum', [model_name '/Sum_zs'], ...
    'Inputs', '++', 'Position', [770, 50, 790, 80]);
add_line(model_name, 'State_Demux/1', 'Sum_zs/1');
add_line(model_name, 'Sum_zus/1', 'Sum_zs/2');

% Mux for 3-signal displacement scope
add_block('simulink/Signal Routing/Mux', [model_name '/Mux_Displacements'], ...
    'Inputs', '3', 'Position', [830, 45, 835, 115]);
add_line(model_name, 'Road_Switch/1', 'Mux_Displacements/1');
add_line(model_name, 'Sum_zus/1', 'Mux_Displacements/2');
add_line(model_name, 'Sum_zs/1', 'Mux_Displacements/3');

%% 7. Scopes & To-Workspace Loggers
% Main 3-Signal Displacement Scope (zr, zus, zs)
add_block('simulink/Sinks/Scope', [model_name '/Displacements_Scope_zr_zus_zs'], ...
    'Position', [870, 65, 910, 95]);
add_line(model_name, 'Mux_Displacements/1', 'Displacements_Scope_zr_zus_zs/1');

% Vehicle Body Acceleration Scope
add_block('simulink/Sinks/Scope', [model_name '/Vehicle_Acceleration_Scope'], ...
    'Position', [890, 230, 930, 260]);
add_line(model_name, 'Sum_zs_ddot/1', 'Vehicle_Acceleration_Scope/1');

% Suspension Deflection Scope
add_block('simulink/Sinks/Scope', [model_name '/Suspension_Deflection_Scope'], ...
    'Position', [890, 140, 930, 170]);
add_line(model_name, 'State_Demux/1', 'Suspension_Deflection_Scope/1');

% Actuator Force Scope
add_block('simulink/Sinks/Scope', [model_name '/Actuator_Control_Force_Scope'], ...
    'Position', [520, 585, 560, 615]);
add_line(model_name, 'Actuator_Saturation/1', 'Actuator_Control_Force_Scope/1');

% Road Profile Scope
add_block('simulink/Sinks/Scope', [model_name '/Road_Profile_Scope'], ...
    'Position', [220, 150, 260, 180]);
add_line(model_name, 'Road_Switch/1', 'Road_Profile_Scope/1');

% Workspace Loggers
add_block('simulink/Sinks/To Workspace', [model_name '/Log_zr'], ...
    'VariableName', 'zr_sim', 'SaveFormat', 'Array', 'Position', [220, 110, 280, 130]);
add_line(model_name, 'Road_Switch/1', 'Log_zr/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_zus'], ...
    'VariableName', 'zus_sim', 'SaveFormat', 'Array', 'Position', [770, 95, 830, 115]);
add_line(model_name, 'Sum_zus/1', 'Log_zus/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_zs'], ...
    'VariableName', 'zs_sim', 'SaveFormat', 'Array', 'Position', [820, 10, 880, 30]);
add_line(model_name, 'Sum_zs/1', 'Log_zs/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_zs_ddot'], ...
    'VariableName', 'zs_ddot_sim', 'SaveFormat', 'Array', 'Position', [890, 280, 960, 300]);
add_line(model_name, 'Sum_zs_ddot/1', 'Log_zs_ddot/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_susp_deflection'], ...
    'VariableName', 'susp_defl_sim', 'SaveFormat', 'Array', 'Position', [800, 175, 870, 195]);
add_line(model_name, 'State_Demux/1', 'Log_susp_deflection/1');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_Fc_force'], ...
    'VariableName', 'Fc_force_sim', 'SaveFormat', 'Array', 'Position', [520, 535, 590, 555]);
add_line(model_name, 'Actuator_Saturation/1', 'Log_Fc_force/1');

save_system(model_name, fullfile(pwd, [model_name '.slx']));
close_system(model_name, 0);

fprintf('Pure Native Simulink Model %s.slx built successfully with full Quanser scope suite!\n', model_name);
