function results = run_candidates(road_id)
% RUN_CANDIDATES Runs and compares all 5 tuning candidates for a selected road profile
%
% Usage:
%   run_candidates       % Runs Road 4 (Bumpy road) by default and displays comparison plot
%   run_candidates(1)    % Runs 5 candidates on Road 1 (Smooth road)
%   run_candidates(2)    % Runs 5 candidates on Road 2 (Normal road - Resonance)
%   run_candidates(3)    % Runs 5 candidates on Road 3 (Rough road - Chirp)
%   run_candidates(4)    % Runs 5 candidates on Road 4 (Bumpy road)
%   run_candidates(5)    % Runs 5 candidates on Road 5 (Pulse road)
%   run_candidates(6)    % Runs 5 candidates on Road 6 (Potholes road)
%   run_candidates('all')% Runs all 6 road profiles (same as run_tuning_experiments)

    proj_dir = fileparts(mfilename('fullpath'));
    if isempty(proj_dir), proj_dir = pwd; end
    cd(proj_dir);
    addpath(genpath(proj_dir));

    if nargin < 1 || isempty(road_id)
        road_id = 4; % Default to Bumpy road
    end

    if ischar(road_id) && strcmpi(road_id, 'all')
        run_tuning_experiments;
        return;
    end

    road_num = road_id;
    if ~ismember(road_num, 1:6)
        error('Invalid road_id %d. Please select an integer from 1 to 6.', road_num);
    end

    road_names = { ...
        'Smooth road (4 mm, 0.2 Hz)', ...
        'Normal road (Resonance 2.58 Hz)', ...
        'Rough road (0.1-15 Hz Chirp)', ...
        'Bumpy road (20 mm Ramp)', ...
        'Pulse road (20 mm Step Drops)', ...
        'Potholes road (ISO Class D + Dips)' ...
    };

    % Candidate definition for all 6 roads
    candidates = cell(6, 1);
    candidates{1} = struct( ...
        'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Cand 5)'}, ...
        'c1',   {3.0, 5.0, 7.0, 10.0, 5.0}, ...
        'k1',   {10.0, 8.0, 60.0, 100.0, 20.0}, ...
        'k2',   {4.0, 2.0, 25.0, 40.0, 8.0} ...
    );
    candidates{2} = struct( ...
        'name', {'Low Gains (Weak)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Cand 5)'}, ...
        'c1',   {4.0, 10.0, 7.0, 10.0, 6.5}, ...
        'k1',   {30.0, 18.0, 60.0, 120.0, 55.0}, ...
        'k2',   {12.0, 7.0, 25.0, 50.0, 22.0} ...
    );
    candidates{3} = struct( ...
        'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Cand 5)'}, ...
        'c1',   {4.5, 12.0, 7.0, 12.0, 6.8}, ...
        'k1',   {35.0, 20.0, 60.0, 140.0, 62.0}, ...
        'k2',   {14.0, 8.0, 25.0, 60.0, 25.0} ...
    );
    candidates{4} = struct( ...
        'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Cand 5)'}, ...
        'c1',   {4.0, 8.0, 7.0, 7.0, 5.8}, ...
        'k1',   {25.0, 13.0, 60.0, 70.0, 45.0}, ...
        'k2',   {10.0, 4.5, 25.0, 30.0, 18.0} ...
    );
    candidates{5} = struct( ...
        'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Cand 5)'}, ...
        'c1',   {5.0, 13.5, 7.0, 14.0, 7.8}, ...
        'k1',   {40.0, 22.0, 60.0, 160.0, 75.0}, ...
        'k2',   {16.0, 9.0, 25.0, 75.0, 30.0} ...
    );
    candidates{6} = struct( ...
        'name', {'Low Gains (Soft)', 'Grant Table', 'Fixed SMC Ref', 'Aggressive (Iter 1)', 'Precision (Cand 5)'}, ...
        'c1',   {5.5, 15.0, 7.0, 15.0, 8.0}, ...
        'k1',   {45.0, 25.0, 60.0, 180.0, 78.0}, ...
        'k2',   {18.0, 10.0, 25.0, 90.0, 32.0} ...
    );

    model_name = 'Master_Active_Suspension';
    if ~bdIsLoaded(model_name), load_system(model_name); end

    fprintf('========================================================================================\n');
    fprintf('  RUNNING ALL 5 TUNING CANDIDATES: %s\n', road_names{road_num});
    fprintf('========================================================================================\n\n');

    try, set_param(model_name, 'FastRestart', 'off'); catch; end
    assignin('base', 'ROAD_ID', road_num);
    assignin('base', 'OVERRIDE_ADAP_GAINS', false);
    evalin('base', 'parameters;');
    assignin('base', 'OVERRIDE_ADAP_GAINS', true);
    try, set_param(model_name, 'FastRestart', 'on'); catch; end

    cand_list = candidates{road_num};
    n_cand = length(cand_list);
    res_list = cell(n_cand, 1);

    fprintf('%-24s | %-4s | %-5s | %-4s | %-10s | %-11s | %-9s | %-10s | %-10s | %-10s\n', ...
        'Candidate Name', 'c1', 'k1', 'k2', 'SS RMS Acc', 'Tot RMS Acc', 'Peak Acc', 'Peak Stroke', 'Peak Force', 'Status');
    fprintf('-------------------------+------+-------+------+------------+-------------+-----------+------------+------------+----------\n');

    for c = 1:n_cand
        c1_v = cand_list(c).c1;
        k1_v = cand_list(c).k1;
        k2_v = cand_list(c).k2;
        name_v = cand_list(c).name;

        assignin('base', 'c1_adap', c1_v);
        assignin('base', 'k1_adap', k1_v);
        assignin('base', 'k2_adap', k2_v);

        simOut = sim(model_name);

        t = simOut.tout(:);
        t_ss = (t >= t(end) * 0.50);

        acc_a = simOut.zs_ddot_asmc(:);
        def_a = simOut.defl_asmc(:) * 1000.0; % mm
        u_a   = simOut.u_asmc(:);

        m = struct();
        m.name       = name_v;
        m.c1         = c1_v;
        m.k1         = k1_v;
        m.k2         = k2_v;
        m.rms_acc_ss = sqrt(mean(acc_a(t_ss).^2));
        m.rms_acc_tot= sqrt(mean(acc_a.^2));
        m.pk_acc     = max(abs(acc_a));
        m.pk_def     = max(abs(def_a));
        m.pk_u       = max(abs(u_a));
        m.t          = t;
        m.acc        = acc_a;
        m.def        = def_a;
        m.u          = u_a;

        if m.pk_def <= 38.001 && m.pk_u <= 39.201
            m.status = 'PASS SAFE';
        else
            m.status = 'VIOLATION';
        end

        res_list{c} = m;

        fprintf('%-24s | %4.1f | %5.1f | %4.1f | %10.4f | %11.4f | %9.3f | %8.2f mm | %8.2f N | %s\n', ...
            name_v, c1_v, k1_v, k2_v, m.rms_acc_ss, m.rms_acc_tot, m.pk_acc, m.pk_def, m.pk_u, m.status);
    end
    fprintf('========================================================================================\n\n');

    try, set_param(model_name, 'FastRestart', 'off'); catch; end

    % Plot side-by-side comparison figure
    fig = figure('Name', sprintf('Candidate Comparison: %s', road_names{road_num}), ...
                 'Position', [100, 100, 1100, 750], 'Color', 'w');

    c_map = [ ...
        0.55, 0.55, 0.55;  % Low Gains (Gray)
        0.85, 0.45, 0.10;  % Grant Table (Orange)
        0.80, 0.15, 0.15;  % Fixed SMC Ref (Red)
        0.60, 0.20, 0.70;  % Aggressive (Purple)
        0.10, 0.65, 0.25   % Candidate 5 Precision (Green)
    ];

    names = cellfun(@(x) x.name, res_list, 'UniformOutput', false);

    % Subplot 1: Steady-state RMS Acceleration
    subplot(2, 2, 1);
    ss_acc = cellfun(@(x) x.rms_acc_ss, res_list);
    b1 = bar(ss_acc, 'FaceColor', 'flat');
    b1.CData = c_map;
    set(gca, 'XTick', 1:n_cand, 'XTickLabel', names, 'XTickLabelRotation', 20);
    ylabel('SS RMS Acc (m/s^2) [Lower is Better]', 'FontWeight', 'bold');
    title('Ride Comfort: Body Acceleration', 'FontSize', 11);
    grid on;

    % Subplot 2: Peak Suspension Stroke vs 38 mm limit
    subplot(2, 2, 2);
    pk_d = cellfun(@(x) x.pk_def, res_list);
    b2 = bar(pk_d, 'FaceColor', 'flat');
    b2.CData = c_map;
    hold on;
    yline(38.0, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Travel Limit (+/- 38 mm)');
    set(gca, 'XTick', 1:n_cand, 'XTickLabel', names, 'XTickLabelRotation', 20);
    ylabel('Peak Stroke (mm)', 'FontWeight', 'bold');
    title('Suspension Travel vs Hard Limit', 'FontSize', 11);
    grid on;
    legend('show', 'Location', 'northeast', 'FontSize', 8);

    % Subplot 3: Peak Actuator Force vs 39.2 N limit
    subplot(2, 2, 3);
    pk_force = cellfun(@(x) x.pk_u, res_list);
    b3 = bar(pk_force, 'FaceColor', 'flat');
    b3.CData = c_map;
    hold on;
    yline(39.2, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Motor Limit (+/- 39.2 N)');
    set(gca, 'XTick', 1:n_cand, 'XTickLabel', names, 'XTickLabelRotation', 20);
    ylabel('Peak Force (N)', 'FontWeight', 'bold');
    title('Actuator Motor Force', 'FontSize', 11);
    grid on;
    legend('show', 'Location', 'northeast', 'FontSize', 8);

    % Subplot 4: Pareto Trade-off
    subplot(2, 2, 4);
    scatter(pk_d, ss_acc, 100, c_map, 'filled', 'MarkerEdgeColor', 'k');
    hold on;
    for i = 1:n_cand
        text(pk_d(i) + 0.15, ss_acc(i), sprintf(' %s', names{i}), 'FontSize', 8, 'FontWeight', 'bold');
    end
    xlabel('Peak Stroke (mm) [Must be < 38 mm]', 'FontWeight', 'bold');
    ylabel('SS RMS Acc (m/s^2) [Lower is Better]', 'FontWeight', 'bold');
    title('Pareto Frontier: Comfort vs Travel', 'FontSize', 11);
    grid on;

    sgtitle(sprintf('Comparison of All 5 Tuning Candidates: %s', road_names{road_num}), 'FontSize', 13, 'FontWeight', 'bold');

    if nargout > 0
        results = res_list;
    end
end
