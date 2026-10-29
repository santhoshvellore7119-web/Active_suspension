function results = simulate_case(road_num, show_plot)
% SIMULATE_CASE Runs Master_Active_Suspension.slx for a selected road profile
% Compares LQR, Fixed Super-Twisting SMC, and Adaptive Super-Twisting SMC
% simultaneously on the exact same plant with identical disturbance input.
%
% 6 Standard Road Profiles from Professor Specifications:
%   1 : Smooth road   (4 mm, 0.20 Hz Sine Wave)
%   2 : Normal road   (Harmonic Near Body Resonance 2.58 Hz)
%   3 : Rough road    (0.1 - 15.0 Hz Frequency Chirp Sweep)
%   4 : Bumpy road    (Triangular Ramp Obstacle)
%   5 : Pulse road    (20 mm Step Pulse Drops)
%   6 : Potholes road (Broadband ISO Class D + Deep Potholes)
%
% Usage:
%   simulate_case()         % Runs Road 1: Smooth road
%   simulate_case(1)        % Runs Road 1: Smooth road
%   simulate_case(2)        % Runs Road 2: Normal road
%   simulate_case(3)        % Runs Road 3: Rough road
%   simulate_case(4)        % Runs Road 4: Bumpy road
%   simulate_case(5)        % Runs Road 5: Pulse road
%   simulate_case(6)        % Runs Road 6: Potholes road

    % Ensure current directory is the project directory
    proj_dir = fileparts(mfilename('fullpath'));
    if ~isempty(proj_dir) && ~strcmp(pwd, proj_dir)
        cd(proj_dir);
        addpath(genpath(proj_dir));
    end

    if nargin < 1 || isempty(road_num)
        road_num = 1; % Default: Smooth road
    end
    if nargin < 2 || isempty(show_plot)
        show_plot = true;
    end

    if ~ismember(road_num, 1:6)
        error('Invalid road_num %d. Please select an integer from 1 to 6.', road_num);
    end

    road_names = { ...
        'Smooth road', ...
        'Normal road', ...
        'Rough road', ...
        'Bumpy road', ...
        'Pulse road', ...
        'Potholes road' ...
    };

    fprintf('\n========================================================================================\n');
    fprintf('  RUNNING 3-CONTROLLER BENCHMARK: %s\n', road_names{road_num});
    fprintf('  Architecture: Parallel Simulink Model (Optimal LQR vs Fixed STSMC vs Adaptive STSMC)\n');
    fprintf('========================================================================================\n\n');

    % Set active road in workspace and load parameters
    assignin('base', 'ROAD_ID', road_num);
    evalin('base', 'parameters;');

    if bdIsLoaded('Master_Active_Suspension')
        model_name = 'Master_Active_Suspension';
    elseif bdIsLoaded('Master_Active_Suspension_Clean')
        model_name = 'Master_Active_Suspension_Clean';
    elseif exist('Master_Active_Suspension_Clean.slx', 'file')
        try
            copyfile('Master_Active_Suspension_Clean.slx', 'Master_Active_Suspension.slx');
            model_name = 'Master_Active_Suspension';
        catch
            model_name = 'Master_Active_Suspension_Clean';
        end
        load_system(model_name);
    else
        model_name = 'Master_Active_Suspension';
        load_system(model_name);
    end

    % Run simulation
    simOut = sim(model_name);

    % Extract time and disturbance
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

    % Ensure consistent lengths
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

    % Performance evaluation window: Steady-state window (t >= 50% duration)
    t_settle = t(end) * 0.50;
    idx_ss = (t >= t_settle);

    calc_metrics = @(acc, defl, tir, u) struct( ...
        'rms_acc_tot', sqrt(mean(acc.^2)), ...
        'rms_acc_ss',  sqrt(mean(acc(idx_ss).^2)), ...
        'pk_acc',      max(abs(acc)), ...
        'rms_def_tot', sqrt(mean(defl.^2)), ...
        'rms_def_ss',  sqrt(mean(defl(idx_ss).^2)), ...
        'pk_def',      max(abs(defl)), ...
        'rms_tir_tot', sqrt(mean(tir.^2)), ...
        'rms_tir_ss',  sqrt(mean(tir(idx_ss).^2)), ...
        'pk_tir',      max(abs(tir)), ...
        'pk_u',        max(abs(u)), ...
        'rms_u_tot',   sqrt(mean(u.^2)), ...
        'rms_u_ss',    sqrt(mean(u(idx_ss).^2)), ...
        'acc',         acc, ...
        'def',         defl, ...
        'tire',        tir, ...
        'u',           u ...
    );

    results = struct();
    results.time     = t;
    results.zr       = zr;
    results.t_settle = t_settle;
    results.road_id  = road_num;
    results.road_name= road_names{road_num};
    results.pass     = calc_metrics(acc_pass, def_pass, tir_pass, u_pass);
    results.lqr      = calc_metrics(acc_lqr,  def_lqr,  tir_lqr,  u_lqr);
    results.fsmc     = calc_metrics(acc_fsmc, def_fsmc, tir_fsmc, u_fsmc);
    results.asmc     = calc_metrics(acc_asmc, def_asmc, tir_asmc, u_asmc);

    % Print formatted table
    fprintf('------------------------------------------------------------------------------------------------------------------------\n');
    fprintf('%-20s | %-10s | %-11s | %-9s | %-10s | %-11s | %-10s | %-17s\n', ...
        'Controller', 'SS RMS Acc', 'Tot RMS Acc', 'Peak Acc', 'SS Stroke', 'Peak Stroke', 'Peak Force', 'Limit Compliance');
    fprintf('%-20s | %-10s | %-11s | %-9s | %-10s | %-11s | %-10s | %-17s\n', ...
        '', '(m/s^2)', '(m/s^2)', '(m/s^2)', '(mm)', '(mm)', '(N)', '[Stroke & Force]');
    fprintf('---------------------+------------+-------------+-----------+------------+-------------+------------+--------------------\n');

    ctrl_list = {'Optimal LQR', 'Fixed STSMC', 'Adaptive STSMC'};
    res_list  = {results.lqr, results.fsmc, results.asmc};

    for i = 1:length(ctrl_list)
        c = res_list{i};
        pass_str = '[PASS SAFE]';
        if c.pk_def > 38.0 || c.pk_u > 39.201
            pass_str = '[VIOLATION]';
        end
        fprintf('%d. %-17s | %10.4f | %11.4f | %9.4f | %10.3f | %11.3f | %10.2f |    %-17s\n', ...
            i, ctrl_list{i}, c.rms_acc_ss, c.rms_acc_tot, c.pk_acc, c.rms_def_ss, c.pk_def, c.pk_u, pass_str);
    end
    fprintf('------------------------------------------------------------------------------------------------------------------------\n');
    fprintf('Hardware Limit Compliance:\n');
    fprintf('  - Rail Travel Limit (+/- 38.0 mm): LQR [%s] | Fixed SMC [%s] | Adaptive SMC [%s]\n', ...
        check_lim(results.lqr.pk_def, 38.0), check_lim(results.fsmc.pk_def, 38.0), check_lim(results.asmc.pk_def, 38.0));
    fprintf('  - Motor Force Limit (+/- 39.2 N) : LQR [%s] | Fixed SMC [%s] | Adaptive SMC [%s]\n', ...
        check_lim(results.lqr.pk_u, 39.2), check_lim(results.fsmc.pk_u, 39.2), check_lim(results.asmc.pk_u, 39.2));
    fprintf('========================================================================================================================\n\n');

    if show_plot
        plot_comparisons(results, road_names{road_num}, road_num);
    end
end

function str = check_lim(val, lim)
    if val <= lim + 0.001
        str = 'PASS [SAFE]';
    else
        str = 'EXCEEDED';
    end
end
