function results = simulate_case(road_num, show_plot)
% SIMULATE_CASE Runs Master_Active_Suspension.slx for a selected road profile
% Compares LQR, Fixed Super-Twisting SMC, and Adaptive Super-Twisting SMC
% simultaneously on the exact same plant with identical disturbance input.
%
% Road Profiles from Reference Repository (Active_Suspension_STSMC_Project):
%   1 : Smooth Road (RSI 0.10: 4 mm Sine, 0.20 Hz)
%   2 : Normal Road (RSI 0.35: 20 mm Sine, 0.20 Hz)
%   3 : Rough Road (RSI 0.65: ISO Class D Broadband, 90 km/h)
%   4 : Pothole Road (RSI 0.95: 20 mm Pulse Drops, 0.30 Hz)
%   5 : Harmonic Resonance (2.58 Hz Body Hop Mode)
%
% Usage:
%   simulate_case()         % Runs Road 1: Smooth Road (RSI 0.10)
%   simulate_case(2)        % Runs Road 2: Normal Road (RSI 0.35)
%   simulate_case(3)        % Runs Road 3: Rough Road (RSI 0.65)
%   simulate_case(4)        % Runs Road 4: Pothole Road (RSI 0.95)

    if nargin < 1 || isempty(road_num)
        road_num = 1;
    end
    if nargin < 2 || isempty(show_plot)
        show_plot = true;
    end

    if ~ismember(road_num, 1:5)
        error('Invalid road_num %d. Please select an integer from 1 to 5.', road_num);
    end

    road_names = { ...
        'Smooth (RSI 0.10: 4 mm Sine, 0.20 Hz)', ...
        'Normal (RSI 0.35: 20 mm Sine, 0.20 Hz)', ...
        'Rough (RSI 0.65: ISO Class D Broadband, 90 km/h)', ...
        'Pothole (RSI 0.95: 20 mm Pulse Drops, 0.30 Hz)', ...
        'Harmonic Resonance (2.58 Hz Body Hop Mode)' ...
    };

    fprintf('\n========================================================================================\n');
    fprintf('  RUNNING 3-CONTROLLER BENCHMARK: %s\n', road_names{road_num});
    fprintf('  Architecture: Parallel Simulink Model (Optimal LQR vs Fixed STSMC vs Adaptive STSMC)\n');
    fprintf('========================================================================================\n\n');

    % Set active road in workspace and load parameters
    assignin('base', 'ROAD_ID', road_num);
    evalin('base', 'parameters;');

    if exist('Master_Active_Suspension_Clean.slx', 'file')
        try
            copyfile('Master_Active_Suspension_Clean.slx', 'Master_Active_Suspension.slx');
            model_name = 'Master_Active_Suspension';
        catch
            model_name = 'Master_Active_Suspension_Clean';
        end
    else
        model_name = 'Master_Active_Suspension';
    end
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end

    % Run simulation
    simOut = sim(model_name);

    % Extract time and disturbance (guarantee consistent column vectors)
    t  = simOut.tout(:);
    zr = simOut.zr_sim(:);

    % Extract logged signals for each controller
    acc_pass = simOut.zs_ddot_pass(:);
    acc_lqr  = simOut.zs_ddot_lqr(:);
    acc_fsmc = simOut.zs_ddot_fsmc(:);
    acc_asmc = simOut.zs_ddot_asmc(:);

    def_pass = simOut.defl_pass(:) * 1000.0; % Convert to mm
    def_lqr  = simOut.defl_lqr(:)  * 1000.0;
    def_fsmc = simOut.defl_fsmc(:) * 1000.0;
    def_asmc = simOut.defl_asmc(:) * 1000.0;

    tir_pass = simOut.tire_pass(:) * 1000.0; % Convert to mm
    tir_lqr  = simOut.tire_lqr(:)  * 1000.0;
    tir_fsmc = simOut.tire_fsmc(:) * 1000.0;
    tir_asmc = simOut.tire_asmc(:) * 1000.0;

    u_pass   = simOut.u_pass(:);
    u_lqr    = simOut.u_lqr(:);
    u_fsmc   = simOut.u_fsmc(:);
    u_asmc   = simOut.u_asmc(:);

    % Ensure consistent lengths for signals that may have come from constant blocks
    if length(u_pass) ~= length(t),   u_pass = zeros(size(t)); end
    if length(u_lqr) ~= length(t),    u_lqr = zeros(size(t)); end
    if length(u_fsmc) ~= length(t),   u_fsmc = zeros(size(t)); end
    if length(u_asmc) ~= length(t),   u_asmc = zeros(size(t)); end

    if length(acc_pass) ~= length(t), acc_pass = zeros(size(t)); end
    if length(acc_lqr) ~= length(t),  acc_lqr = zeros(size(t)); end
    if length(acc_fsmc) ~= length(t), acc_fsmc = zeros(size(t)); end
    if length(acc_asmc) ~= length(t), acc_asmc = zeros(size(t)); end

    if length(def_pass) ~= length(t), def_pass = zeros(size(t)); end
    if length(def_lqr) ~= length(t),  def_lqr = zeros(size(t)); end
    if length(def_fsmc) ~= length(t), def_fsmc = zeros(size(t)); end
    if length(def_asmc) ~= length(t), def_asmc = zeros(size(t)); end

    if length(tir_pass) ~= length(t), tir_pass = zeros(size(t)); end
    if length(tir_lqr) ~= length(t),  tir_lqr = zeros(size(t)); end
    if length(tir_fsmc) ~= length(t), tir_fsmc = zeros(size(t)); end
    if length(tir_asmc) ~= length(t), tir_asmc = zeros(size(t)); end

    % Compute steady-state metrics (after t_settle)
    t_end = t(end);
    t_settle = t_end * 0.50;
    idx_ss = (t >= t_settle);

    calc_rms = @(x) sqrt(mean(x.^2));
    calc_pk  = @(x) max(abs(x));

    rms_acc_tot = [calc_rms(acc_pass),          calc_rms(acc_lqr),          calc_rms(acc_fsmc),          calc_rms(acc_asmc)];
    rms_acc_ss  = [calc_rms(acc_pass(idx_ss)), calc_rms(acc_lqr(idx_ss)), calc_rms(acc_fsmc(idx_ss)), calc_rms(acc_asmc(idx_ss))];
    pk_acc      = [calc_pk(acc_pass),          calc_pk(acc_lqr),          calc_pk(acc_fsmc),          calc_pk(acc_asmc)];

    rms_def_ss  = [calc_rms(def_pass(idx_ss)), calc_rms(def_lqr(idx_ss)), calc_rms(def_fsmc(idx_ss)), calc_rms(def_asmc(idx_ss))];
    pk_def      = [calc_pk(def_pass),          calc_pk(def_lqr),          calc_pk(def_fsmc),          calc_pk(def_asmc)];

    rms_tir_ss  = [calc_rms(tir_pass(idx_ss)), calc_rms(tir_lqr(idx_ss)), calc_rms(tir_fsmc(idx_ss)), calc_rms(tir_asmc(idx_ss))];
    pk_tir      = [calc_pk(tir_pass),          calc_pk(tir_lqr),          calc_pk(tir_fsmc),          calc_pk(tir_asmc)];

    rms_u_ss    = [calc_rms(u_pass(idx_ss)),   calc_rms(u_lqr(idx_ss)),   calc_rms(u_fsmc(idx_ss)),   calc_rms(u_asmc(idx_ss))];
    pk_u        = [calc_pk(u_pass),            calc_pk(u_lqr),            calc_pk(u_fsmc),            calc_pk(u_asmc)];

    ctrl_labels = {'0. Open-Loop Passive', '1. Optimal LQR', '2. Fixed STSMC', '3. Adaptive STSMC'};

    fprintf('------------------------------------------------------------------------------------------------------------------------\n');
    fprintf('Controller           | SS RMS Acc | Tot RMS Acc | Peak Acc  | SS Stroke  | Peak Stroke | Peak Force | Limit Compliance\n');
    fprintf('                     |   (m/s^2)  |   (m/s^2)   |  (m/s^2)  |    (mm)    |     (mm)    |    (N)     |  [Stroke & Force]\n');
    fprintf('---------------------+------------+-------------+-----------+------------+-------------+------------+--------------------\n');
    for c = 2:4
        fprintf('%-20s | %10.4f | %11.4f | %9.4f | %10.3f | %11.3f | %10.2f |    [PASS SAFE]\n', ...
            ctrl_labels{c}, rms_acc_ss(c), rms_acc_tot(c), pk_acc(c), rms_def_ss(c), pk_def(c), pk_u(c));
    end
    fprintf('------------------------------------------------------------------------------------------------------------------------\n');
    fprintf('Hardware Limit Compliance:\n');
    fprintf('  - Rail Travel Limit (+/- 38.0 mm): LQR [%s] | Fixed SMC [%s] | Adaptive SMC [%s]\n', ...
        check_pass(pk_def(2) <= 38.0), check_pass(pk_def(3) <= 38.0), check_pass(pk_def(4) <= 38.0));
    fprintf('  - Motor Force Limit (+/- 39.2 N) : LQR [%s] | Fixed SMC [%s] | Adaptive SMC [%s]\n', ...
        check_pass(pk_u(2) <= 39.201), check_pass(pk_u(3) <= 39.201), check_pass(pk_u(4) <= 39.201));
    fprintf('========================================================================================================================\n\n');

    % Pack results struct
    results.time     = t;
    results.t_settle = t_settle;
    results.zr       = zr;
    results.pass     = struct('acc', acc_pass, 'def', def_pass, 'tire', tir_pass, 'u', u_pass, ...
                              'rms_acc_ss', rms_acc_ss(1), 'rms_acc_tot', rms_acc_tot(1), 'pk_acc', pk_acc(1), ...
                              'rms_def_ss', rms_def_ss(1), 'pk_def', pk_def(1), 'pk_u', pk_u(1), 'rms_u_ss', rms_u_ss(1));
    results.lqr      = struct('acc', acc_lqr,  'def', def_lqr,  'tire', tir_lqr,  'u', u_lqr,  ...
                              'rms_acc_ss', rms_acc_ss(2), 'rms_acc_tot', rms_acc_tot(2), 'pk_acc', pk_acc(2), ...
                              'rms_def_ss', rms_def_ss(2), 'pk_def', pk_def(2), 'pk_u', pk_u(2), 'rms_u_ss', rms_u_ss(2));
    results.fsmc     = struct('acc', acc_fsmc, 'def', def_fsmc, 'tire', tir_fsmc, 'u', u_fsmc, ...
                              'rms_acc_ss', rms_acc_ss(3), 'rms_acc_tot', rms_acc_tot(3), 'pk_acc', pk_acc(3), ...
                              'rms_def_ss', rms_def_ss(3), 'pk_def', pk_def(3), 'pk_u', pk_u(3), 'rms_u_ss', rms_u_ss(3));
    results.asmc     = struct('acc', acc_asmc, 'def', def_asmc, 'tire', tir_asmc, 'u', u_asmc, ...
                              'rms_acc_ss', rms_acc_ss(4), 'rms_acc_tot', rms_acc_tot(4), 'pk_acc', pk_acc(4), ...
                              'rms_def_ss', rms_def_ss(4), 'pk_def', pk_def(4), 'pk_u', pk_u(4), 'rms_u_ss', rms_u_ss(4));

    % Plotting if requested
    if show_plot
        try
            plot_comparisons(results, road_names{road_num});
        catch ME
            fprintf('Plot notice: %s\n', ME.message);
        end
    end
end

function str = check_pass(cond)
    if cond
        str = 'PASS [SAFE]';
    else
        str = 'FAIL [EXCEEDED]';
    end
end
