function results = run_candidate5(road_id, show_plot)
% RUN_CANDIDATE5 Runs Candidate 5 (Precision-Tuned Adaptive STSMC) for all road profiles
% Evaluates the official gain-scheduled controller across all 6 standardized profiles.
%
% Usage:
%   run_candidate5                 % Runs Candidate 5 on all 6 road profiles (with plot)
%   run_candidate5('all', false)   % Runs Candidate 5 on all 6 road profiles without plots
%   run_candidate5(4, false)       % Runs Candidate 5 on Road 4 without plots

    proj_dir = fileparts(mfilename('fullpath'));
    if isempty(proj_dir), proj_dir = pwd; end
    cd(proj_dir);
    addpath(genpath(proj_dir));

    if nargin < 1 || isempty(road_id) || (ischar(road_id) && strcmpi(road_id, 'all'))
        run_roads = 1:6;
    elseif ischar(road_id)
        run_roads = str2double(road_id);
    else
        run_roads = road_id;
    end

    if nargin < 2 || isempty(show_plot)
        show_plot = true;
    end

    road_names = { ...
        'Smooth road (4 mm, 0.20 Hz)', ...
        'Normal road (Resonance 2.58 Hz)', ...
        'Rough road (0.1-15 Hz Chirp)', ...
        'Bumpy road (20 mm Ramp)', ...
        'Pulse road (20 mm Step Drops)', ...
        'Potholes road (ISO Class D + Dips)' ...
    };

    model_name = 'Master_Active_Suspension';
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end

    fprintf('========================================================================================================\n');
    fprintf('  RUNNING CANDIDATE 5 (PRECISION-TUNED ADAPTIVE STSMC) ACROSS %d ROAD PROFILE(S)\n', length(run_roads));
    fprintf('  Architecture: Parallel Simulink Benchmark (Master_Active_Suspension.slx)\n');
    fprintf('========================================================================================================\n\n');

    fprintf('%-3s | %-28s | %-4s | %-5s | %-4s | %-10s | %-10s | %-11s | %-10s | %-10s\n', ...
        'ID', 'Road Profile Name', 'c1', 'k1', 'k2', 'SS RMS Acc', 'Peak Acc', 'Peak Stroke', 'Peak Force', 'Compliance');
    fprintf('----+------------------------------+------+-------+------+------------+------------+-------------+------------+-----------\n');

    res_all = cell(length(run_roads), 1);

    for idx = 1:length(run_roads)
        r = run_roads(idx);
        
        % Ensure FastRestart is OFF between roads
        try, set_param(model_name, 'FastRestart', 'off'); catch; end
        
        assignin('base', 'ROAD_ID', r);
        assignin('base', 'OVERRIDE_ADAP_GAINS', false); % Uses Candidate 5 (default tuned scaled schedule)
        evalin('base', 'parameters;');
        
        % Read the scheduled Candidate 5 gains
        c1_v = evalin('base', 'c1_adap');
        k1_v = evalin('base', 'k1_adap');
        k2_v = evalin('base', 'k2_adap');
        
        simOut = sim(model_name);
        
        t = simOut.tout(:);
        t_ss = (t >= t(end) * 0.50);
        
        acc_asmc = simOut.zs_ddot_asmc(:);
        def_asmc = simOut.defl_asmc(:) * 1000.0; % mm
        u_asmc   = simOut.u_asmc(:);
        
        % Extract baselines for comparison
        acc_fsmc = simOut.zs_ddot_fsmc(:);
        acc_lqr  = simOut.zs_ddot_lqr(:);
        def_fsmc = simOut.defl_fsmc(:) * 1000.0;
        def_lqr  = simOut.defl_lqr(:) * 1000.0;
        
        m = struct();
        m.road_id     = r;
        m.road_name   = road_names{r};
        m.c1          = c1_v;
        m.k1          = k1_v;
        m.k2          = k2_v;
        m.rms_acc_ss  = sqrt(mean(acc_asmc(t_ss).^2));
        m.rms_acc_tot = sqrt(mean(acc_asmc.^2));
        m.pk_acc      = max(abs(acc_asmc));
        m.rms_def_ss  = sqrt(mean(def_asmc(t_ss).^2));
        m.pk_def      = max(abs(def_asmc));
        m.pk_u        = max(abs(u_asmc));
        m.stroke_margin = ((38.0 - m.pk_def) / 38.0) * 100.0;
        m.force_margin  = ((39.2 - m.pk_u) / 39.2) * 100.0;
        
        m.lqr_rms_ss  = sqrt(mean(acc_lqr(t_ss).^2));
        m.fsmc_rms_ss = sqrt(mean(acc_fsmc(t_ss).^2));
        m.lqr_pk_def  = max(abs(def_lqr));
        m.fsmc_pk_def = max(abs(def_fsmc));
        
        if m.pk_def <= 38.001 && m.pk_u <= 39.201
            m.status = 'PASS [SAFE]';
        else
            m.status = 'VIOLATION';
        end
        
        res_all{idx} = m;
        
        fprintf('%2d  | %-28s | %4.1f | %5.1f | %4.1f | %8.4f m/s^2 | %7.3f m/s^2 | %7.2f mm  | %7.2f N  | %s\n', ...
            r, road_names{r}, c1_v, k1_v, k2_v, m.rms_acc_ss, m.pk_acc, m.pk_def, m.pk_u, m.status);
    end
    fprintf('========================================================================================================\n\n');

    % Render multi-road comparative dashboard if plotting is enabled
    if show_plot
        if length(run_roads) == 6
            fig = figure('Name', 'Candidate 5 (Precision Tuned STSMC) Multi-Road Performance', ...
                         'Position', [100, 100, 1150, 700], 'Color', 'w');
            
            % Subplot 1: Steady-State RMS Acceleration vs Road Profiles
            subplot(2, 2, 1);
            ss_asmc = cellfun(@(x) x.rms_acc_ss, res_all);
            ss_fsmc = cellfun(@(x) x.fsmc_rms_ss, res_all);
            ss_lqr  = cellfun(@(x) x.lqr_rms_ss, res_all);
            
            b = bar([ss_lqr, ss_fsmc, ss_asmc]);
            b(1).FaceColor = [0.15, 0.35, 0.75]; % LQR (Blue)
            b(2).FaceColor = [0.80, 0.20, 0.20]; % Fixed STSMC (Red)
            b(3).FaceColor = [0.10, 0.65, 0.25]; % Cand 5 Adaptive (Green)
            set(gca, 'XTick', 1:6, 'XTickLabel', {'1. Smooth', '2. Normal', '3. Rough', '4. Bumpy', '5. Pulse', '6. Potholes'}, 'XTickLabelRotation', 15);
            ylabel('SS RMS Acc (m/s^2) [Lower is Better]', 'FontWeight', 'bold');
            title('Ride Comfort Benchmark across 6 Road Profiles', 'FontSize', 11);
            legend({'Optimal LQR', 'Fixed STSMC', 'Candidate 5 (Adaptive STSMC)'}, 'Location', 'northwest', 'FontSize', 8);
            grid on;
            
            % Subplot 2: Peak Suspension Stroke vs 38 mm limit
            subplot(2, 2, 2);
            pk_asmc = cellfun(@(x) x.pk_def, res_all);
            b2 = bar(pk_asmc, 'FaceColor', [0.10, 0.65, 0.25]);
            hold on;
            yline(38.0, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Travel Limit (+/- 38 mm)');
            yline(38.0 * 0.75, 'm--', 'LineWidth', 1.2, 'DisplayName', '75% Safety Threshold');
            set(gca, 'XTick', 1:6, 'XTickLabel', {'1. Smooth', '2. Normal', '3. Rough', '4. Bumpy', '5. Pulse', '6. Potholes'}, 'XTickLabelRotation', 15);
            ylabel('Peak Stroke (mm)', 'FontWeight', 'bold');
            title('Candidate 5 Suspension Travel vs 38 mm Hard Limit', 'FontSize', 11);
            legend({'Candidate 5 Stroke', 'Hard Limit (38 mm)', 'Warning Threshold'}, 'Location', 'northwest', 'FontSize', 8);
            grid on;
            
            % Subplot 3: Peak Actuator Force vs 39.2 N limit
            subplot(2, 2, 3);
            pk_u = cellfun(@(x) x.pk_u, res_all);
            b3 = bar(pk_u, 'FaceColor', [0.10, 0.65, 0.25]);
            hold on;
            yline(39.2, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Motor Saturation (+/- 39.2 N)');
            set(gca, 'XTick', 1:6, 'XTickLabel', {'1. Smooth', '2. Normal', '3. Rough', '4. Bumpy', '5. Pulse', '6. Potholes'}, 'XTickLabelRotation', 15);
            ylabel('Peak Force (N)', 'FontWeight', 'bold');
            title('Candidate 5 Actuator Demand vs 39.2 N Motor Limit', 'FontSize', 11);
            legend({'Candidate 5 Force', 'Motor Saturation'}, 'Location', 'northwest', 'FontSize', 8);
            grid on;
            
            % Subplot 4: Adaptive Gains (c1, k1, k2) vs Road Severity
            subplot(2, 2, 4);
            k1_vals = cellfun(@(x) x.k1, res_all);
            k2_vals = cellfun(@(x) x.k2, res_all);
            c1_vals = cellfun(@(x) x.c1, res_all);
            
            yyaxis left
            plot(1:6, k1_vals, 'b-o', 'LineWidth', 2.0, 'MarkerSize', 6, 'DisplayName', 'k_1 Convergence');
            hold on;
            plot(1:6, k2_vals, 'r-s', 'LineWidth', 2.0, 'MarkerSize', 6, 'DisplayName', 'k_2 Robustness');
            ylabel('Gains k_1 and k_2', 'FontWeight', 'bold');
            
            yyaxis right
            plot(1:6, c1_vals, 'g-^', 'LineWidth', 2.0, 'MarkerSize', 6, 'DisplayName', 'c_1 Sliding Slope');
            ylabel('Sliding Slope c_1', 'FontWeight', 'bold');
            
            set(gca, 'XTick', 1:6, 'XTickLabel', {'1. Smooth', '2. Normal', '3. Rough', '4. Bumpy', '5. Pulse', '6. Potholes'}, 'XTickLabelRotation', 15);
            title('Candidate 5 Scheduled Gains Across Road Profiles', 'FontSize', 11);
            legend('Location', 'northwest', 'FontSize', 8);
            grid on;
            
            sgtitle('Candidate 5 (Precision-Tuned Adaptive STSMC) Comprehensive Benchmark', 'FontSize', 13, 'FontWeight', 'bold');
            
            out_plot = fullfile(proj_dir, 'results', 'Figure_Candidate5_All_Roads.png');
            exportgraphics(fig, out_plot, 'Resolution', 300);
            art_dir = 'C:\Users\Santhosh\.gemini\antigravity\brain\9d9628c5-7574-49b5-b04e-877bd3c4aab2';
            copyfile(out_plot, fullfile(art_dir, 'Figure_Candidate5_All_Roads.png'));
            fprintf('Saved summary dashboard plot: %s\n\n', out_plot);
        elseif length(run_roads) == 1
            % Single road: open detailed 3-controller dynamic comparison plot
            simulate_case(run_roads(1), true);
        end
    end

    if nargout > 0
        results = res_all;
    end
end
