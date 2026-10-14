function sim_results = simulate_road(road_num, controller_type)
% SIMULATE_ROAD Run active suspension simulation for any of the 6 road profiles in one command.
%
% Usage:
%   simulate_road()                   % Runs default profile (2: Smooth Highway)
%   simulate_road(3)                  % Runs Road Profile 3 (Severe Potholes)
%   simulate_road(6, 'STSMC')         % Runs Road Profile 6 (Resonance) with Adaptive STSMC
%   simulate_road(3, 'LQR')           % Runs Road Profile 3 with Optimal LQR
%   simulate_road(3, 'PASSIVE')       % Runs Road Profile 3 with Passive Suspension
%
% Available Road Profiles:
%   1 : Original Quanser Harmonics / Frequency Chirp (0.1 - 10 Hz)
%   2 : Smooth Undulating Highway (4 mm Sine, 0.20 Hz)
%   3 : Severe Discrete Potholes (-20 mm Pulse Drops)
%   4 : Triangular Ramp Obstacle & Speed Bump (+20 mm Wedge)
%   5 : Real-World Stochastic Rough Terrain + Shocks (15 mm broadband)
%   6 : Harmonic Corrugated Resonance Road (10 mm, 2.58 Hz Body Hop Mode)
%
% Available Controllers:
%   'STSMC'    : Super-Twisting Sliding Mode Controller (Default, Adaptive)
%   'LQR'      : Linear Quadratic Regulator (Optimal ACO-tuned)
%   'LQR_CLASS': Classical Quanser Baseline LQR
%   'PASSIVE'  : Passive Uncontrolled Baseline (Fc = 0)

    if nargin < 1 || isempty(road_num)
        road_num = 2;
    end
    if nargin < 2 || isempty(controller_type)
        controller_type = 'STSMC';
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
    fprintf('  RUNNING QUANSER ACTIVE SUSPENSION: ONE-COMMAND SIMULATION\n');
    fprintf('  Selected Road: %s\n', road_names{road_num});
    fprintf('  Selected Controller: %s\n', upper(controller_type));
    fprintf('========================================================================\n\n');

    % Set workspace parameters for Simulink
    assignin('base', 'ROAD_PROFILE_ID', road_num);
    
    switch upper(controller_type)
        case 'PASSIVE'
            assignin('base', 'CONTROLLER_ID', 1);
        case {'LQR', 'LQR_ACO'}
            assignin('base', 'CONTROLLER_ID', 3);
            assignin('base', 'LQR_TYPE', 'ACO');
        case 'LQR_CLASS'
            assignin('base', 'CONTROLLER_ID', 2);
            assignin('base', 'LQR_TYPE', 'CLASSICAL');
        otherwise % 'STSMC'
            assignin('base', 'CONTROLLER_ID', 6);
            assignin('base', 'STSMC_MODE', 'adaptive');
    end

    evalin('base', 'setup_as;');

    model_name = 'Master_Active_Suspension_Simulation';
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end

    % Run simulation
    simOut = sim(model_name);

    % Extract simulation logs
    t = simOut.tout;
    zs_def = simOut.susp_defl_sim;
    zs_ddot = simOut.zs_ddot_sim;
    fc = simOut.Fc_force_sim;
    zr = simOut.zr_sim;

    % Calculate RMS and Peak metrics
    rms_def = sqrt(mean(zs_def.^2)) * 1000;      % mm
    peak_def = max(abs(zs_def)) * 1000;          % mm
    rms_acc = sqrt(mean(zs_ddot.^2));            % m/s^2
    peak_acc = max(abs(zs_ddot));                % m/s^2
    rms_fc = sqrt(mean(fc.^2));                  % N
    peak_fc = max(abs(fc));                      % N

    % Hardware limits validation
    def_margin = (1 - (peak_def / 38.0)) * 100;
    fc_margin = (1 - (peak_fc / 39.2)) * 100;

    fprintf('\n------------------------------------------------------------------------\n');
    fprintf('  PERFORMANCE & RMS COMPARATIVE RESULTS\n');
    fprintf('------------------------------------------------------------------------\n');
    fprintf('  Suspension Travel (Deflection):  RMS = %6.2f mm  |  Peak = %6.2f mm\n', rms_def, peak_def);
    fprintf('  Sprung Mass Acceleration:        RMS = %6.2f m/s^2| Peak = %6.2f m/s^2\n', rms_acc, peak_acc);
    fprintf('  Actuator Control Force:          RMS = %6.2f N   |  Peak = %6.2f N\n', rms_fc, peak_fc);
    fprintf('------------------------------------------------------------------------\n');
    fprintf('  Hardware Safety Checks:\n');
    fprintf('    - Mechanical Rail Limit (+/- 38 mm) : %s (Peak: %4.1f mm, Margin: %4.1f%%)\n', ...
        check_str(peak_def <= 38.0), peak_def, def_margin);
    fprintf('    - Motor Force Limit     (+/- 39.2 N): %s (Peak: %4.1f N,  Margin: %4.1f%%)\n', ...
        check_str(peak_fc <= 39.2001), peak_fc, fc_margin);
    fprintf('========================================================================\n\n');

    sim_results.time = t;
    sim_results.zr = zr;
    sim_results.susp_deflection = zs_def;
    sim_results.zs_ddot = zs_ddot;
    sim_results.fc = fc;
    sim_results.metrics = struct( ...
        'rms_def_mm', rms_def, 'peak_def_mm', peak_def, ...
        'rms_acc', rms_acc, 'peak_acc', peak_acc, ...
        'rms_fc', rms_fc, 'peak_fc', peak_fc ...
    );
end

function s = check_str(pass)
    if pass
        s = 'PASS [SAFE]';
    else
        s = 'FAIL [EXCEEDED]';
    end
end
