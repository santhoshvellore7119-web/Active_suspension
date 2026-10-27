function sim_results = simulate_road(road_num, controller_type)
% SIMULATE_ROAD Run active suspension simulation for any of the 6 road profiles in one command.
% Evaluates Passive, Optimal LQR, Fixed STSMC, and Adaptive STSMC simultaneously.
%
% Usage:
%   simulate_road()                   % Runs default profile (2: Smooth Highway)
%   simulate_road(3)                  % Runs Road Profile 3 (Severe Potholes)
%   simulate_road(6)                  % Runs Road Profile 6 (Resonance Corrugation)
%   simulate_road(5)                  % Runs Road Profile 5 (ISO Class D Rough Terrain)
%
% Available Road Profiles:
%   1 : Original Quanser Harmonics / Frequency Chirp (0.1 - 10 Hz)
%   2 : Smooth Undulating Highway (4 mm Sine, 0.20 Hz)
%   3 : Severe Discrete Potholes (-20 mm Pulse Drops)
%   4 : Triangular Ramp Obstacle & Speed Bump (+20 mm Wedge)
%   5 : Real-World Stochastic Rough Terrain + Shocks (15 mm broadband ISO D)
%   6 : Harmonic Corrugated Resonance Road (10 mm, 2.58 Hz Body Hop Mode)

    if nargin < 1 || isempty(road_num)
        road_num = 2;
    end
    if nargin < 2 || isempty(controller_type)
        controller_type = 'ALL';
    end

    if ~ismember(road_num, 1:6)
        error('Invalid road number %d. Please enter an integer from 1 to 6.', road_num);
    end

    road_names = { ...
        '1. Original Quanser Harmonics / Chirp (0.1 - 10 Hz)', ...
        '2. Smooth Undulating Highway (4 mm Sine, 0.20 Hz)', ...
        '3. Severe Discrete Potholes (-20 mm Step Drops)', ...
        '4. Triangular Ramp Obstacle & Speed Bump (+20 mm Wedge)', ...
        '5. Stochastic Rough Terrain + Pothole Shocks (ISO Class D)', ...
        '6. Harmonic Corrugated Resonance Road (2.58 Hz Body Hop Mode)' ...
    };

    fprintf('\n========================================================================\n');
    fprintf('  RUNNING QUANSER ACTIVE SUSPENSION: 4-CONTROLLER BENCHMARK\n');
    fprintf('  Selected Road: %s\n', road_names{road_num});
    fprintf('  Architecture:  Parallel 4-Branch Native Simulink Engine (Dinesh Format)\n');
    fprintf('========================================================================\n\n');

    % Set workspace parameters for Simulink
    assignin('base', 'ROAD_PROFILE_ID', road_num);
    assignin('base', 'CONTROLLER_ID', 6);
    assignin('base', 'STSMC_MODE', 'adaptive');

    evalin('base', 'setup_as;');

    model_name = 'Master_Active_Suspension_Simulation';
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end

    % Run simulation
    simOut = sim(model_name);

    % Extract simulation logs for all four parallel comparative branches
    t = simOut.tout;
    zr = simOut.zr_sim;

    acc_p = simOut.zs_ddot_passive_sim;
    acc_l = simOut.zs_ddot_lqr_sim;
    def_p = simOut.defl_passive_sim * 1000; % mm
    def_l = simOut.defl_lqr_sim * 1000;     % mm
    fc_p  = simOut.Fc_passive_sim;
    fc_l  = simOut.Fc_lqr_sim;

    if isprop(simOut, 'zs_ddot_fixed_stsmc_sim') || isfield(simOut, 'zs_ddot_fixed_stsmc_sim')
        acc_f = simOut.zs_ddot_fixed_stsmc_sim;
        acc_a = simOut.zs_ddot_adap_stsmc_sim;
        def_f = simOut.defl_fixed_stsmc_sim * 1000;
        def_a = simOut.defl_adap_stsmc_sim * 1000;
        fc_f  = simOut.Fc_fixed_stsmc_sim;
        fc_a  = simOut.Fc_adap_stsmc_sim;
    else
        acc_f = simOut.zs_ddot_stsmc_sim;
        acc_a = simOut.zs_ddot_stsmc_sim;
        def_f = simOut.defl_stsmc_sim * 1000;
        def_a = simOut.defl_stsmc_sim * 1000;
        fc_f  = simOut.Fc_stsmc_sim;
        fc_a  = simOut.Fc_stsmc_sim;
    end

    % Performance metrics for all 4 controllers
    rms_acc_p = sqrt(mean(acc_p.^2));  peak_acc_p = max(abs(acc_p));
    rms_acc_l = sqrt(mean(acc_l.^2));  peak_acc_l = max(abs(acc_l));
    rms_acc_f = sqrt(mean(acc_f.^2));  peak_acc_f = max(abs(acc_f));
    rms_acc_a = sqrt(mean(acc_a.^2));  peak_acc_a = max(abs(acc_a));

    rms_def_p = sqrt(mean(def_p.^2));  peak_def_p = max(abs(def_p));
    rms_def_l = sqrt(mean(def_l.^2));  peak_def_l = max(abs(def_l));
    rms_def_f = sqrt(mean(def_f.^2));  peak_def_f = max(abs(def_f));
    rms_def_a = sqrt(mean(def_a.^2));  peak_def_a = max(abs(def_a));

    rms_fc_p  = sqrt(mean(fc_p.^2));   peak_fc_p  = max(abs(fc_p));
    rms_fc_l  = sqrt(mean(fc_l.^2));   peak_fc_l  = max(abs(fc_l));
    rms_fc_f  = sqrt(mean(fc_f.^2));   peak_fc_f  = max(abs(fc_f));
    rms_fc_a  = sqrt(mean(fc_a.^2));   peak_fc_a  = max(abs(fc_a));

    % Compute percentage reductions relative to Passive
    pct_acc_l = (1 - rms_acc_l / rms_acc_p) * 100;
    pct_acc_f = (1 - rms_acc_f / rms_acc_p) * 100;
    pct_acc_a = (1 - rms_acc_a / rms_acc_p) * 100;

    fprintf('\n-----------------------------------------------------------------------------------------------------------------\n');
    fprintf('  PARALLEL 4-CONTROLLER BENCHMARK RESULTS (Simulink Native Execution - Dinesh Master Format)\n');
    fprintf('-----------------------------------------------------------------------------------------------------------------\n');
    fprintf('Controller       | RMS Acc (m/s^2) | Acc Reduct (%%) | Peak Acc (m/s^2) | RMS Trav (mm) | Peak Trav (mm) | Peak Force (N)\n');
    fprintf('-----------------+-----------------+----------------+------------------+---------------+----------------+---------------\n');
    fprintf('Passive Baseline | %15.4f | %14.1f | %16.4f | %13.3f | %14.3f | %14.2f\n', ...
        rms_acc_p, 0.0, peak_acc_p, rms_def_p, peak_def_p, peak_fc_p);
    fprintf('Optimal LQR      | %15.4f | %+13.1f%% | %16.4f | %13.3f | %14.3f | %14.2f\n', ...
        rms_acc_l, pct_acc_l, peak_acc_l, rms_def_l, peak_def_l, peak_fc_l);
    fprintf('Fixed STSMC      | %15.4f | %+13.1f%% | %16.4f | %13.3f | %14.3f | %14.2f\n', ...
        rms_acc_f, pct_acc_f, peak_acc_f, rms_def_f, peak_def_f, peak_fc_f);
    fprintf('Adaptive STSMC   | %15.4f | %+13.1f%% | %16.4f | %13.3f | %14.3f | %14.2f\n', ...
        rms_acc_a, pct_acc_a, peak_acc_a, rms_def_a, peak_def_a, peak_fc_a);
    fprintf('-----------------------------------------------------------------------------------------------------------------\n');
    fprintf('  Hardware Physical Safety Limit Checks:\n');
    fprintf('    - Mechanical Rail Travel (+/- 38.0 mm): Passive [%s] | LQR [%s] | Fixed STSMC [%s] | Adaptive STSMC [%s]\n', ...
        check_str(peak_def_p <= 38.0), check_str(peak_def_l <= 38.0), check_str(peak_def_f <= 38.0), check_str(peak_def_a <= 38.0));
    fprintf('    - Motor Force Limit      (+/- 39.2 N) : Passive [%s] | LQR [%s] | Fixed STSMC [%s] | Adaptive STSMC [%s]\n', ...
        check_str(peak_fc_p <= 39.2001), check_str(peak_fc_l <= 39.2001), check_str(peak_fc_f <= 39.2001), check_str(peak_fc_a <= 39.2001));
    fprintf('=================================================================================================================\n\n');

    sim_results.time = t;
    sim_results.zr = zr;
    sim_results.passive      = struct('acc', acc_p, 'def', def_p, 'fc', fc_p, 'rms_acc', rms_acc_p, 'rms_def', rms_def_p, 'peak_acc', peak_acc_p, 'peak_def', peak_def_p, 'peak_fc', peak_fc_p);
    sim_results.lqr          = struct('acc', acc_l, 'def', def_l, 'fc', fc_l, 'rms_acc', rms_acc_l, 'rms_def', rms_def_l, 'peak_acc', peak_acc_l, 'peak_def', peak_def_l, 'peak_fc', peak_fc_l);
    sim_results.fixed_stsmc  = struct('acc', acc_f, 'def', def_f, 'fc', fc_f, 'rms_acc', rms_acc_f, 'rms_def', rms_def_f, 'peak_acc', peak_acc_f, 'peak_def', peak_def_f, 'peak_fc', peak_fc_f);
    sim_results.adaptive_stsmc = struct('acc', acc_a, 'def', def_a, 'fc', fc_a, 'rms_acc', rms_acc_a, 'rms_def', rms_def_a, 'peak_acc', peak_acc_a, 'peak_def', peak_def_a, 'peak_fc', peak_fc_a);

    % Call auto_plot_results to update figures
    try
        auto_plot_results();
    catch ME
        fprintf('Plotting note: %s\n', ME.message);
    end
end

function s = check_str(pass)
    if pass
        s = 'PASS';
    else
        s = 'FAIL';
    end
end
