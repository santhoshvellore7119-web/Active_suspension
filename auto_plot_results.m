% AUTO_PLOT_RESULTS
% Automatic Post-Simulation Plotting Function
% Invoked automatically by Simulink StopFcn callback or called manually.
% Generates publication-ready comparative plots comparing:
%   1) Passive Baseline
%   2) Optimal LQR
%   3) Fixed STSMC
%   4) Adaptive STSMC

function auto_plot_results()
    % Check if simulation data exists in base workspace
    has_signals = evalin('base', 'exist(''zs_ddot_pass'', ''var'') || exist(''zs_ddot_lqr'', ''var'')');
    
    if ~has_signals
        % Check if simOut exists in workspace
        if evalin('base', 'exist(''simOut'', ''var'')')
            simOut = evalin('base', 'simOut');
            if ismember('zs_ddot_pass', simOut.who)
                % Extract into workspace or simulate_case format
                t = simOut.tout(:);
                zr = simOut.zr_sim(:);
            else
                return;
            end
        else
            return;
        end
    end

    try
        road_id = 1;
        if evalin('base', 'exist(''ROAD_ID'', ''var'')')
            road_id = evalin('base', 'ROAD_ID');
        elseif evalin('base', 'exist(''ROAD_PROFILE_ID'', ''var'')')
            road_id = evalin('base', 'ROAD_PROFILE_ID');
        end
        
        road_names = { ...
            'Smooth Undulating Highway (4 mm Sine, 0.20 Hz)', ...
            'Severe Discrete Potholes (-20 mm Pulse Drops)', ...
            'Dynamic Frequency-Sweep Chirp (0.1 - 10 Hz)', ...
            'Stochastic ISO Class D Rough Terrain (90 km/h)', ...
            'Harmonic Corrugated Resonance Road (2.58 Hz Body Hop)' ...
        };
        road_str = road_names{max(1, min(5, road_id))};

        % Extract vectors
        if evalin('base', 'exist(''tout'', ''var'')')
            t = evalin('base', 'tout(:)');
        else
            t = linspace(0, 10, 10001)';
        end

        idx_ss = (t >= t(end)*0.50);
        calc_rms = @(x) sqrt(mean(x.^2));

        % Build results struct
        res = struct();
        res.time = t;
        res.t_settle = t(end)*0.50;
        
        if evalin('base', 'exist(''zr_sim'', ''var'')')
            res.zr = evalin('base', 'zr_sim(:)');
        else
            res.zr = zeros(size(t));
        end

        % Passive
        if evalin('base', 'exist(''zs_ddot_pass'', ''var'')')
            acc_p = evalin('base', 'zs_ddot_pass(:)');
            def_p = evalin('base', 'defl_pass(:)') * 1000;
            res.pass = struct('acc', acc_p, 'def', def_p, 'u', zeros(size(t)), ...
                'rms_acc_ss', calc_rms(acc_p(idx_ss)), 'pk_def', max(abs(def_p)), 'pk_u', 0);
        else
            res.pass = struct('acc', zeros(size(t)), 'def', zeros(size(t)), 'u', zeros(size(t)), ...
                'rms_acc_ss', 0.001, 'pk_def', 0, 'pk_u', 0);
        end

        % LQR
        if evalin('base', 'exist(''zs_ddot_lqr'', ''var'')')
            acc_l = evalin('base', 'zs_ddot_lqr(:)');
            def_l = evalin('base', 'defl_lqr(:)') * 1000;
            u_l   = evalin('base', 'u_lqr(:)');
            rms_l = calc_rms(acc_l(idx_ss));
            iso_l = (1 - rms_l / max(res.pass.rms_acc_ss, 1e-4)) * 100;
            res.lqr = struct('acc', acc_l, 'def', def_l, 'u', u_l, ...
                'rms_acc_ss', rms_l, 'pk_def', max(abs(def_l)), 'pk_u', max(abs(u_l)), 'iso_pct', iso_l);
        end

        % Fixed STSMC
        if evalin('base', 'exist(''zs_ddot_fsmc'', ''var'')')
            acc_f = evalin('base', 'zs_ddot_fsmc(:)');
            def_f = evalin('base', 'defl_fsmc(:)') * 1000;
            u_f   = evalin('base', 'u_fsmc(:)');
            rms_f = calc_rms(acc_f(idx_ss));
            iso_f = (1 - rms_f / max(res.pass.rms_acc_ss, 1e-4)) * 100;
            res.fsmc = struct('acc', acc_f, 'def', def_f, 'u', u_f, ...
                'rms_acc_ss', rms_f, 'pk_def', max(abs(def_f)), 'pk_u', max(abs(u_f)), 'iso_pct', iso_f);
        end

        % Adaptive STSMC
        if evalin('base', 'exist(''zs_ddot_asmc'', ''var'')')
            acc_a = evalin('base', 'zs_ddot_asmc(:)');
            def_a = evalin('base', 'defl_asmc(:)') * 1000;
            u_a   = evalin('base', 'u_asmc(:)');
            rms_a = calc_rms(acc_a(idx_ss));
            iso_a = (1 - rms_a / max(res.pass.rms_acc_ss, 1e-4)) * 100;
            res.asmc = struct('acc', acc_a, 'def', def_a, 'u', u_a, ...
                'rms_acc_ss', rms_a, 'pk_def', max(abs(def_a)), 'pk_u', max(abs(u_a)), 'iso_pct', iso_a);
        end

        plot_comparisons(res, road_str);
    catch ME
        fprintf('Auto plot notice: %s\n', ME.message);
    end
end
