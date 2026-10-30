% RUN_TUNING_EXPERIMENTS.M
% Comprehensive multi-candidate parameter tuning study for Adaptive STSMC
% Quanser Quarter-Car Active Suspension System Model (QC-ASSM)
% Evaluates 5 distinct (c1, k1, k2) candidates across all 6 road profiles (30 simulation runs)
% Analyzes Future Road Profile Scope (continuous RSI mapping)
% Generates publication-grade sensitivity figures and exports full numerical data

fprintf('========================================================================================\n');
fprintf('  STARTING COMPREHENSIVE STSMC PARAMETER TUNING STUDY & ROAD BENCHMARK\n');
fprintf('  Quanser 2-DOF Active Suspension: Multi-Candidate Evaluation Across All Profiles\n');
fprintf('========================================================================================\n\n');

proj_dir = fileparts(mfilename('fullpath'));
if isempty(proj_dir), proj_dir = pwd; end
cd(proj_dir);
addpath(genpath(proj_dir));

out_dir = fullfile(proj_dir, 'results', 'tuning_curves');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

art_dir = 'C:\Users\Santhosh\.gemini\antigravity\brain\9d9628c5-7574-49b5-b04e-877bd3c4aab2';

% Ensure Simulink model is loaded
model_name = 'Master_Active_Suspension';
if ~bdIsLoaded(model_name)
    load_system(model_name);
end

road_names = { ...
    'Smooth road', ...
    'Normal road', ...
    'Rough road', ...
    'Bumpy road', ...
    'Pulse road', ...
    'Potholes road' ...
};

% Enable gain override flag
assignin('base', 'OVERRIDE_ADAP_GAINS', true);

% Define Candidates for each Road Profile
candidates = cell(6, 1);

% Road 1: Smooth road (RSI = 0.10)
candidates{1} = struct( ...
    'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Iter 2)'}, ...
    'c1',   {3.0, 5.0, 7.0, 10.0, 5.0}, ...
    'k1',   {10.0, 8.0, 60.0, 100.0, 20.0}, ...
    'k2',   {4.0, 2.0, 25.0, 40.0, 8.0} ...
);

% Road 2: Normal road (RSI = 0.65)
candidates{2} = struct( ...
    'name', {'Low Gains (Weak)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Iter 2)'}, ...
    'c1',   {4.0, 10.0, 7.0, 10.0, 6.5}, ...
    'k1',   {30.0, 18.0, 60.0, 120.0, 55.0}, ...
    'k2',   {12.0, 7.0, 25.0, 50.0, 22.0} ...
);

% Road 3: Rough road (RSI = 0.75)
candidates{3} = struct( ...
    'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Iter 2)'}, ...
    'c1',   {4.5, 12.0, 7.0, 12.0, 6.8}, ...
    'k1',   {35.0, 20.0, 60.0, 140.0, 62.0}, ...
    'k2',   {14.0, 8.0, 25.0, 60.0, 25.0} ...
);

% Road 4: Bumpy road (RSI = 0.40)
candidates{4} = struct( ...
    'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Iter 2)'}, ...
    'c1',   {4.0, 8.0, 7.0, 7.0, 5.8}, ...
    'k1',   {25.0, 13.0, 60.0, 70.0, 45.0}, ...
    'k2',   {10.0, 4.5, 25.0, 30.0, 18.0} ...
);

% Road 5: Pulse road (RSI = 0.85)
candidates{5} = struct( ...
    'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Iter 2)'}, ...
    'c1',   {5.0, 13.5, 7.0, 14.0, 7.8}, ...
    'k1',   {40.0, 22.0, 60.0, 160.0, 75.0}, ...
    'k2',   {16.0, 9.0, 25.0, 75.0, 30.0} ...
);

% Road 6: Potholes road (RSI = 0.95)
candidates{6} = struct( ...
    'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Iter 2)'}, ...
    'c1',   {5.5, 15.0, 7.0, 15.0, 8.0}, ...
    'k1',   {45.0, 25.0, 60.0, 180.0, 78.0}, ...
    'k2',   {18.0, 10.0, 25.0, 90.0, 32.0} ...
);

all_results = cell(6, 5);
fixed_results = cell(6, 1);
lqr_results   = cell(6, 1);

total_sims = 6 * 5;
sim_counter = 0;

for r = 1:6
    fprintf('\n>>> PROCESSING PROFILE %d: %s <<<\n', r, road_names{r});
    
    % Ensure FastRestart is OFF when setting up new road
    try, set_param(model_name, 'FastRestart', 'off'); catch; end
    
    assignin('base', 'ROAD_ID', r);
    assignin('base', 'OVERRIDE_ADAP_GAINS', false); % Load default for road
    evalin('base', 'parameters;');
    assignin('base', 'OVERRIDE_ADAP_GAINS', true);  % Enable override for sweep
    
    % Turn ON FastRestart for fast candidate iterations on this road
    try, set_param(model_name, 'FastRestart', 'on'); catch; end
    
    cand_list = candidates{r};
    n_cand = length(cand_list);
    cand_data = cell(n_cand, 1);
    
    for c = 1:n_cand
        sim_counter = sim_counter + 1;
        c1_val = cand_list(c).c1;
        k1_val = cand_list(c).k1;
        k2_val = cand_list(c).k2;
        cand_name = cand_list(c).name;
        
        assignin('base', 'c1_adap', c1_val);
        assignin('base', 'k1_adap', k1_val);
        assignin('base', 'k2_adap', k2_val);
        
        t_start = tic;
        simOut = sim(model_name);
        el_time = toc(t_start);
        
        t = simOut.tout(:);
        t_ss = (t >= t(end) * 0.50);
        
        acc_a = simOut.zs_ddot_asmc(:);
        def_a = simOut.defl_asmc(:) * 1000.0; % mm
        u_a   = simOut.u_asmc(:);
        
        metrics = struct();
        metrics.road_id     = r;
        metrics.road_name   = road_names{r};
        metrics.cand_name   = cand_name;
        metrics.c1          = c1_val;
        metrics.k1          = k1_val;
        metrics.k2          = k2_val;
        metrics.rms_acc_ss  = sqrt(mean(acc_a(t_ss).^2));
        metrics.rms_acc_tot = sqrt(mean(acc_a.^2));
        metrics.pk_acc      = max(abs(acc_a));
        metrics.rms_def_ss  = sqrt(mean(def_a(t_ss).^2));
        metrics.pk_def      = max(abs(def_a));
        metrics.pk_u        = max(abs(u_a));
        metrics.stroke_margin = ((38.0 - metrics.pk_def) / 38.0) * 100.0;
        metrics.force_margin  = ((39.2 - metrics.pk_u) / 39.2) * 100.0;
        
        if metrics.pk_def <= 38.001 && metrics.pk_u <= 39.201
            metrics.status = 'PASS SAFE';
        else
            metrics.status = 'VIOLATION';
        end
        
        cand_data{c} = metrics;
        all_results{r, c} = metrics;
        
        if c == 1
            acc_f = simOut.zs_ddot_fsmc(:);
            def_f = simOut.defl_fsmc(:) * 1000.0;
            u_f   = simOut.u_fsmc(:);
            
            f_met = struct();
            f_met.rms_acc_ss  = sqrt(mean(acc_f(t_ss).^2));
            f_met.rms_acc_tot = sqrt(mean(acc_f.^2));
            f_met.pk_acc      = max(abs(acc_f));
            f_met.pk_def      = max(abs(def_f));
            f_met.pk_u        = max(abs(u_f));
            fixed_results{r}  = f_met;
            
            acc_l = simOut.zs_ddot_lqr(:);
            def_l = simOut.defl_lqr(:) * 1000.0;
            u_l   = simOut.u_lqr(:);
            
            l_met = struct();
            l_met.rms_acc_ss  = sqrt(mean(acc_l(t_ss).^2));
            l_met.rms_acc_tot = sqrt(mean(acc_l.^2));
            l_met.pk_acc      = max(abs(acc_l));
            l_met.pk_def      = max(abs(def_l));
            l_met.pk_u        = max(abs(u_l));
            lqr_results{r}    = l_met;
        end
        
        fprintf('  [%2d/%2d] %-20s (c1=%4.1f, k1=%5.1f, k2=%4.1f) -> SS RMS: %7.4f m/s^2 | Pk Stroke: %5.2f mm | Pk Force: %5.2f N [%s] (%.2f s)\n', ...
            sim_counter, total_sims, cand_name, c1_val, k1_val, k2_val, metrics.rms_acc_ss, metrics.pk_def, metrics.pk_u, metrics.status, el_time);
    end
    
    % Turn OFF FastRestart after road candidates finish
    try, set_param(model_name, 'FastRestart', 'off'); catch; end
    
    % Generate multi-candidate comparison figure for this road
    fig = figure('Visible', 'off', 'Position', [100, 100, 1100, 750], 'Color', 'w');
    
    c_map = [ ...
        0.55, 0.55, 0.55;  % Low Gains (Gray)
        0.85, 0.45, 0.10;  % Grant Table (Orange)
        0.80, 0.15, 0.15;  % Fixed SMC Ref (Red)
        0.60, 0.20, 0.70;  % Aggressive (Purple)
        0.10, 0.65, 0.25   % Precision Tuned (Green)
    ];
    
    % Plot 1: Steady-State RMS Acceleration vs Candidates
    subplot(2, 2, 1);
    ss_rms_vals = cellfun(@(m) m.rms_acc_ss, cand_data);
    b1 = bar(ss_rms_vals, 'FaceColor', 'flat');
    b1.CData = c_map;
    hold on;
    yline(fixed_results{r}.rms_acc_ss, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fixed STSMC Baseline');
    yline(lqr_results{r}.rms_acc_ss, 'b:', 'LineWidth', 1.5, 'DisplayName', 'Optimal LQR Baseline');
    set(gca, 'XTick', 1:n_cand, 'XTickLabel', {cand_data{1}.cand_name, cand_data{2}.cand_name, cand_data{3}.cand_name, cand_data{4}.cand_name, cand_data{5}.cand_name}, 'XTickLabelRotation', 20);
    ylabel('Steady-State RMS Acc (m/s^2)', 'FontWeight', 'bold');
    title(sprintf('%s: Ride Comfort Evaluation', road_names{r}), 'FontSize', 11);
    grid on;
    legend({'Candidates', 'Fixed STSMC', 'LQR'}, 'Location', 'northeast', 'FontSize', 8);
    
    % Plot 2: Peak Suspension Stroke vs 38 mm limit
    subplot(2, 2, 2);
    pk_def_vals = cellfun(@(m) m.pk_def, cand_data);
    b2 = bar(pk_def_vals, 'FaceColor', 'flat');
    b2.CData = c_map;
    hold on;
    yline(38.0, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Hard Limit (+/- 38 mm)');
    yline(38.0 * 0.75, 'm--', 'LineWidth', 1.2, 'DisplayName', '75% Warning Margin');
    set(gca, 'XTick', 1:n_cand, 'XTickLabel', {cand_data{1}.cand_name, cand_data{2}.cand_name, cand_data{3}.cand_name, cand_data{4}.cand_name, cand_data{5}.cand_name}, 'XTickLabelRotation', 20);
    ylabel('Peak Suspension Stroke (mm)', 'FontWeight', 'bold');
    title('Mechanical Travel Rattle Space Compliance', 'FontSize', 11);
    grid on;
    legend({'Candidates', 'Hard Limit (38mm)', 'Safety Threshold'}, 'Location', 'northeast', 'FontSize', 8);
    
    % Plot 3: Peak Actuator Force vs 39.2 N limit
    subplot(2, 2, 3);
    pk_u_vals = cellfun(@(m) m.pk_u, cand_data);
    b3 = bar(pk_u_vals, 'FaceColor', 'flat');
    b3.CData = c_map;
    hold on;
    yline(39.2, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Motor Saturation (+/- 39.2 N)');
    set(gca, 'XTick', 1:n_cand, 'XTickLabel', {cand_data{1}.cand_name, cand_data{2}.cand_name, cand_data{3}.cand_name, cand_data{4}.cand_name, cand_data{5}.cand_name}, 'XTickLabelRotation', 20);
    ylabel('Peak Actuator Force (N)', 'FontWeight', 'bold');
    title('Actuator Saturation Envelope', 'FontSize', 11);
    grid on;
    legend({'Candidates', 'Motor Saturation'}, 'Location', 'northeast', 'FontSize', 8);
    
    % Plot 4: Pareto Trade-off Curve (Comfort vs Stroke)
    subplot(2, 2, 4);
    scatter(pk_def_vals, ss_rms_vals, 90, c_map, 'filled', 'MarkerEdgeColor', 'k');
    hold on;
    for c_i = 1:n_cand
        text(pk_def_vals(c_i) + 0.3, ss_rms_vals(c_i), sprintf(' %s', cand_data{c_i}.cand_name), 'FontSize', 7.5, 'FontWeight', 'bold');
    end
    scatter(fixed_results{r}.pk_def, fixed_results{r}.rms_acc_ss, 110, 'd', 'MarkerEdgeColor', 'r', 'LineWidth', 2.0, 'DisplayName', 'Fixed STSMC');
    scatter(lqr_results{r}.pk_def, lqr_results{r}.rms_acc_ss, 110, 's', 'MarkerEdgeColor', 'b', 'LineWidth', 2.0, 'DisplayName', 'Optimal LQR');
    xlabel('Peak Suspension Stroke (mm)', 'FontWeight', 'bold');
    ylabel('SS RMS Acc (m/s^2) [Lower is Better]', 'FontWeight', 'bold');
    title('Pareto Frontier: Comfort vs. Rattle Space', 'FontSize', 11);
    grid on;
    legend('show', 'Location', 'best', 'FontSize', 8);
    
    sgtitle(sprintf('Parameter Tuning Sensitivity & Trade-off Analysis: %s', road_names{r}), 'FontSize', 13, 'FontWeight', 'bold');
    
    plot_file = fullfile(out_dir, sprintf('Tuning_Sensitivity_Road_%d.png', r));
    exportgraphics(fig, plot_file, 'Resolution', 300);
    copyfile(plot_file, fullfile(art_dir, sprintf('Tuning_Sensitivity_Road_%d.png', r)));
    close(fig);
    fprintf('  Saved sensitivity figure: %s\n', plot_file);
end

%% Continuous RSI Scope Analysis (Future Road Profiles)
fprintf('\n>>> EVALUATING CONTINUOUS RSI GAIN SCHEDULER (FUTURE ROAD SCOPE) <<<\n');
rsi_test_points = linspace(0.0, 1.0, 21);
rsi_k1 = zeros(size(rsi_test_points));
rsi_k2 = zeros(size(rsi_test_points));
rsi_c1 = zeros(size(rsi_test_points));

for idx = 1:length(rsi_test_points)
    [k1_out, k2_out, c1_out] = stsmc_rsi_scheduler(rsi_test_points(idx), 'continuous');
    rsi_k1(idx) = k1_out;
    rsi_k2(idx) = k2_out;
    rsi_c1(idx) = c1_out;
end

rsi_scope_data = struct( ...
    'rsi_points', rsi_test_points, ...
    'k1', rsi_k1, ...
    'k2', rsi_k2, ...
    'c1', rsi_c1 ...
);

fig_rsi = figure('Visible', 'off', 'Position', [100, 100, 950, 480], 'Color', 'w');
subplot(1, 2, 1);
plot(rsi_test_points, rsi_k1, 'b-o', 'LineWidth', 2.0, 'MarkerSize', 5, 'DisplayName', 'k_1(RSI) Convergence Gain');
hold on;
plot(rsi_test_points, rsi_k2, 'r-s', 'LineWidth', 2.0, 'MarkerSize', 5, 'DisplayName', 'k_2(RSI) Robustness Integral Gain');
grid on;
xlabel('Continuous Road Severity Index (RSI)', 'FontWeight', 'bold');
ylabel('Controller Gains', 'FontWeight', 'bold');
title('Continuous Adaptive Gain Scheduling Curves', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 9);

subplot(1, 2, 2);
plot(rsi_test_points, rsi_c1, 'g-^', 'LineWidth', 2.0, 'MarkerSize', 5, 'DisplayName', 'c_1(RSI) Sliding Surface Slope');
grid on;
xlabel('Continuous Road Severity Index (RSI)', 'FontWeight', 'bold');
ylabel('Sliding Surface Slope c_1', 'FontWeight', 'bold');
title('Sliding Manifold Parameter c_1(\lambda) vs. RSI', 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 9);

sgtitle('Future Road Profile Scope: Piecewise-Monotonic Continuous Gain Scheduler', 'FontSize', 13, 'FontWeight', 'bold');
rsi_plot_file = fullfile(out_dir, 'Continuous_RSI_Scheduler_Curves.png');
exportgraphics(fig_rsi, rsi_plot_file, 'Resolution', 300);
copyfile(rsi_plot_file, fullfile(art_dir, 'Continuous_RSI_Scheduler_Curves.png'));
close(fig_rsi);
fprintf('  Saved continuous RSI scheduler figure: %s\n', rsi_plot_file);

% Save all experiment data to MAT file
save_file = fullfile(proj_dir, 'results', 'tuning_experiments_data.mat');
save(save_file, 'all_results', 'fixed_results', 'lqr_results', 'candidates', 'rsi_scope_data', 'road_names');
fprintf('\nAll experiment data saved successfully to: %s\n', save_file);
fprintf('========================================================================================\n');
fprintf('  STSMC PARAMETER TUNING STUDY COMPLETED SUCCESSFULLY\n');
fprintf('========================================================================================\n');
