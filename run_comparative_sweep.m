% RUN_COMPARATIVE_SWEEP
% Master Multi-Road & Multi-Controller Comparative Simulation Sweep
% Evaluating LQR (Classical, ACO, HS) vs. Super-Twisting Sliding Mode Control (STSMC)
%
% Compares 6 Controller Configurations across ALL 6 STANDARDIZED ROAD PROFILES (36 Test Runs):
%   Controllers:
%     1. Passive Suspension (Zero Actuator Force Baseline)
%     2. Classical Quanser LQR (Standard Laboratory Feedback)
%     3. Optimal / ACO-Tuned LQR (Riccati ARE Optimal Design)
%     4. Harmony Search (HS) LQR (Dinesh Model Baseline)
%     5. Fixed STSMC (Moderate Fixed Bandwidth)
%     6. Adaptive STSMC (Road Severity Index RSI-Scheduled)
%
%   Road Profiles:
%     Profile 1: Dynamic Frequency-Sweep (Chirp: 0.1 - 10 Hz)         [RSI = 0.50]
%     Profile 2: Smooth Undulating Highway (4 mm Sine, 0.20 Hz)        [RSI = 0.10]
%     Profile 3: Severe Discrete Potholes (20 mm Transient Drops)      [RSI = 0.95]
%     Profile 4: Triangular Ramp & Obstacle / Speed Bump               [RSI = 0.40]
%     Profile 5: Real-World Stochastic Rough Terrain + Pothole Shocks  [RSI = 0.85]
%     Profile 6: Harmonic Corrugated Resonance Road (f ~ 2.58 Hz)     [RSI = 0.65]

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  STARTING COMPREHENSIVE LQR VS. STSMC COMPARATIVE SIMULATION SWEEP\n');
fprintf('  Full 6-Road Profile Evaluation Suite (36 Benchmark Test Runs)\n');
fprintf('========================================================================\n\n');

setup_as;

dt = 0.001;
t_span = 0:dt:T_stop;
N_pts = length(t_span);

% Controller definitions
ctrl_names = {'Passive Baseline', 'Classical LQR', 'ACO-Tuned LQR', 'HS-Tuned LQR', 'Fixed STSMC', 'Adaptive STSMC'};
ctrl_short = {'Passive', 'Class-LQR', 'ACO-LQR', 'HS-LQR', 'Fixed-STSMC', 'Adap-STSMC'};
N_ctrl = length(ctrl_names);

% Road definitions (All 6 Standardized Profiles)
road_names = {
    'Profile 1: Frequency-Sweep Chirp (0.1 - 10 Hz)', ...
    'Profile 2: Smooth Undulating Highway (4 mm Sine, 0.20 Hz)', ...
    'Profile 3: Severe Discrete Potholes (20 mm Pulse Drops)', ...
    'Profile 4: Triangular Ramp Obstacle & Speed Bump (20 mm)', ...
    'Profile 5: Stochastic Rough Road + Potholes (ISO Class D + Shocks)', ...
    'Profile 6: Harmonic Corrugated Resonance Road (f ~ 2.58 Hz)'
};
road_short = {'Chirp', 'Smooth', 'Pothole', 'Ramp', 'Rough+Shock', 'Resonance'};
rsi_vals   = [0.50, 0.10, 0.95, 0.40, 0.85, 0.65];
N_roads    = length(road_names);

% Storage arrays for performance metrics
res_rms_accel = zeros(N_roads, N_ctrl);
res_pk_accel  = zeros(N_roads, N_ctrl);
res_rms_susp  = zeros(N_roads, N_ctrl);
res_pk_susp   = zeros(N_roads, N_ctrl);
res_rms_tire  = zeros(N_roads, N_ctrl);
res_pk_tire   = zeros(N_roads, N_ctrl);
res_pk_force  = zeros(N_roads, N_ctrl);
res_rms_force = zeros(N_roads, N_ctrl);
res_exec_time = zeros(N_roads, N_ctrl);

% Pre-generate Road disturbance signals zr and zr_dot for all 6 profiles
road_zr     = cell(N_roads, 1);
road_zr_dot = cell(N_roads, 1);

% 1. Frequency-Sweep Chirp: 0.1 Hz to 10.0 Hz over 10 s, Amp = 5 mm
k_chirp = (road_chirp_f1 - road_chirp_f0) / T_stop;
phase_chirp = 2 * pi * (road_chirp_f0 * t_span + 0.5 * k_chirp * (t_span.^2));
inst_freq_chirp = 2 * pi * (road_chirp_f0 + k_chirp * t_span);
road_zr{1}     = road_chirp_amp * sin(phase_chirp);
road_zr_dot{1} = road_chirp_amp * inst_freq_chirp .* cos(phase_chirp);

% 2. Smooth Undulating Highway: 4 mm Sine, 0.20 Hz
road_zr{2}     = road_smooth_amp * sin(2*pi*road_smooth_freq * t_span);
road_zr_dot{2} = road_smooth_amp * 2*pi*road_smooth_freq * cos(2*pi*road_smooth_freq * t_span);

% 3. Severe Repetitive Discrete Potholes: 20 mm drops at 0.2, 2.2, 4.2, 6.2, 8.2 s
zr_p = zeros(1, N_pts);
period_pts = round((1/road_pothole_freq) / dt);
pulse_pts  = round(period_pts * (road_pothole_width / 100));
for i = 1:N_pts
    t_curr = t_span(i);
    % Discrete pothole cavities
    if (t_curr >= 0.2 && t_curr < 0.5) || (t_curr >= 2.2 && t_curr < 2.5) || ...
       (t_curr >= 4.2 && t_curr < 4.5) || (t_curr >= 6.2 && t_curr < 6.5) || ...
       (t_curr >= 8.2 && t_curr < 8.5)
        zr_p(i) = -road_pothole_amp;
    else
        zr_p(i) = 0.0;
    end
end
zr_dot_p = zeros(1, N_pts);
for i = 2:N_pts
    zr_dot_p(i) = (zr_p(i) - zr_p(i-1)) / dt;
    zr_dot_p(i) = max(min(zr_dot_p(i), 1.2), -1.2); % Realistic tire compliance slope
end
road_zr{3}     = zr_p;
road_zr_dot{3} = zr_dot_p;

% 4. Triangular Ramp Obstacle & Speed Bump: Incline to +20 mm and sudden drop-off
zr_ramp = zeros(1, N_pts);
for i = 1:N_pts
    t_curr = t_span(i);
    % Bump 1: 0.5 s to 2.0 s
    if t_curr >= 0.5 && t_curr < 1.5
        zr_ramp(i) = road_ramp_height * (t_curr - 0.5) / 1.0; % Incline
    elseif t_curr >= 1.5 && t_curr < 1.52
        zr_ramp(i) = road_ramp_height * (1.0 - (t_curr - 1.5)/0.02); % Abrupt drop
    % Bump 2: 5.0 s to 6.5 s
    elseif t_curr >= 5.0 && t_curr < 6.0
        zr_ramp(i) = road_ramp_height * (t_curr - 5.0) / 1.0;
    elseif t_curr >= 6.0 && t_curr < 6.02
        zr_ramp(i) = road_ramp_height * (1.0 - (t_curr - 6.0)/0.02);
    else
        zr_ramp(i) = 0.0;
    end
end
zr_dot_ramp = zeros(1, N_pts);
for i = 2:N_pts
    zr_dot_ramp(i) = (zr_ramp(i) - zr_ramp(i-1)) / dt;
    zr_dot_ramp(i) = max(min(zr_dot_ramp(i), 1.0), -1.0);
end
road_zr{4}     = zr_ramp;
road_zr_dot{4} = zr_dot_ramp;

% 5. Real-World Stochastic Rough Terrain + Embedded Shocks: ISO Class D + Potholes
rng(42); % Exact reproducibility
n_harmonics = 30;
omega_harm = 2*pi*linspace(0.5, 15.0, n_harmonics);
phi_harm   = rand(1, n_harmonics) * 2 * pi;
amp_harm   = road_rough_amp * (1 ./ (1:n_harmonics).^0.9);
amp_harm   = amp_harm / sum(amp_harm) * road_rough_amp;

zr_rough = zeros(1, N_pts);
zr_dot_rough = zeros(1, N_pts);
for k = 1:n_harmonics
    zr_rough     = zr_rough + amp_harm(k) * sin(omega_harm(k)*t_span + phi_harm(k));
    zr_dot_rough = zr_dot_rough + amp_harm(k) * omega_harm(k) * cos(omega_harm(k)*t_span + phi_harm(k));
end
% Add embedded heavy pothole shocks at t=2.0s, t=5.0s, t=8.0s
for i = 1:N_pts
    t_curr = t_span(i);
    if (t_curr >= 2.0 && t_curr < 2.15) || (t_curr >= 5.0 && t_curr < 5.15) || (t_curr >= 8.0 && t_curr < 8.15)
        zr_rough(i) = zr_rough(i) - road_shock_amp;
    end
end
for i = 2:N_pts
    diff_val = (zr_rough(i) - zr_rough(i-1)) / dt;
    zr_dot_rough(i) = max(min(diff_val, 1.5), -1.5);
end
road_zr{5}     = zr_rough;
road_zr_dot{5} = zr_dot_rough;

% 6. Harmonic Corrugated Resonance Road: Exciting body hop mode at ~2.58 Hz
road_zr{6}     = road_res_amp * sin(2*pi*road_res_freq * t_span);
road_zr_dot{6} = road_res_amp * 2*pi*road_res_freq * cos(2*pi*road_res_freq * t_span);

% Output figures directory
fig_dir = fullfile(pwd, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

fprintf('Executing 36-case benchmark (%d road profiles x %d controllers)...\n\n', N_roads, N_ctrl);

sim_trajectories = cell(N_roads, N_ctrl);

for r_idx = 1:N_roads
    fprintf('========================================================================\n');
    fprintf('  ROAD PROFILE %d: %s (RSI = %.2f)\n', r_idx, road_names{r_idx}, rsi_vals(r_idx));
    fprintf('========================================================================\n');
    zr_curr     = road_zr{r_idx};
    zr_dot_curr = road_zr_dot{r_idx};
    
    for c_idx = 1:N_ctrl
        tic;
        
        x       = zeros(4, N_pts);
        zs_ddot = zeros(1, N_pts);
        Fc      = zeros(1, N_pts);
        w_int   = 0.0;
        
        if c_idx == 5 % Fixed STSMC
            c1_val = c1_fixed; k1_val = k1_fixed; k2_val = k2_fixed;
        elseif c_idx == 6 % Adaptive STSMC
            [k1_val, k2_val, c1_val] = stsmc_rsi_scheduler(r_idx, 'scaled');
        end
        
        % High-precision RK4 integration
        for k = 1:N_pts-1
            x_k = x(:, k);
            zr_dot_k = zr_dot_curr(k);
            
            % Compute Raw Control Force
            switch c_idx
                case 1 % Passive Baseline
                    u = 0.0;
                case 2 % Classical Quanser LQR
                    u = -K_classical * x_k;
                case 3 % Optimal / ACO-Tuned LQR
                    u = -K_aco * x_k;
                case 4 % Harmony Search (HS) LQR
                    u = -K_hs * x_k;
                case {5, 6} % Super-Twisting Sliding Mode Control (STSMC)
                    s = c1_val * x_k(1) + x_k(2);
                    tanh_s = tanh(s / eps_bl);
                    u_prop = -k1_val * sqrt(abs(s)) * tanh_s;
                    w_int = w_int - k2_val * tanh_s * dt;
                    w_int = max(min(w_int, Fc_max), Fc_min);
                    u = u_prop + w_int;
            end
            
            % Physical Actuator Saturation (+/- 39.2 N)
            u_sat = max(min(u, Fc_max), Fc_min);
            Fc(k) = u_sat;
            
            % State derivative: x_dot = A*x + B*[zr_dot; u_sat]
            u_vec = [zr_dot_k; u_sat];
            x_dot = A * x_k + B * u_vec;
            
            % RK4 step
            k1_rk = x_dot;
            k2_rk = A * (x_k + 0.5*dt*k1_rk) + B * u_vec;
            k3_rk = A * (x_k + 0.5*dt*k2_rk) + B * u_vec;
            k4_rk = A * (x_k + dt*k3_rk) + B * u_vec;
            x(:, k+1) = x_k + (dt/6) * (k1_rk + 2*k2_rk + 2*k3_rk + k4_rk);
            
            % True sprung mass body acceleration
            zs_ddot(k) = (-ks * x_k(1) - bs * (x_k(2) - x_k(4)) + u_sat) / ms;
        end
        zs_ddot(N_pts) = zs_ddot(N_pts-1);
        Fc(N_pts)      = Fc(N_pts-1);
        
        sim_time = toc;
        
        % Calculate Performance Metrics
        res_rms_accel(r_idx, c_idx) = rms(zs_ddot);
        res_pk_accel(r_idx, c_idx)  = max(abs(zs_ddot));
        res_rms_susp(r_idx, c_idx)  = rms(x(1, :));
        res_pk_susp(r_idx, c_idx)   = max(abs(x(1, :)));
        res_rms_tire(r_idx, c_idx)  = rms(x(3, :));
        res_pk_tire(r_idx, c_idx)   = max(abs(x(3, :)));
        res_pk_force(r_idx, c_idx)  = max(abs(Fc));
        res_rms_force(r_idx, c_idx) = rms(Fc);
        res_exec_time(r_idx, c_idx) = (sim_time / N_pts) * 1e6; % us per step
        
        sim_trajectories{r_idx, c_idx} = struct(...
            't', t_span, 'x', x, 'zs_ddot', zs_ddot, 'Fc', Fc, 'zr', zr_curr);
        
        safe_travel = res_pk_susp(r_idx, c_idx) <= St_max;
        safe_force  = res_pk_force(r_idx, c_idx) <= (Fc_max + 1e-3);
        status_str  = 'SAFE';
        if ~safe_travel, status_str = 'EXCEEDED STROKE'; end
        if ~safe_force,  status_str = 'EXCEEDED FORCE'; end
        
        fprintf('  [%-16s] RMS Accel: %7.4f m/s^2 | RMS Susp: %7.4f mm | Pk Susp: %6.2f mm [%s] | Pk Force: %5.1f N | Time: %5.1f us\n', ...
            ctrl_names{c_idx}, res_rms_accel(r_idx, c_idx), res_rms_susp(r_idx, c_idx)*1000, ...
            res_pk_susp(r_idx, c_idx)*1000, status_str, res_pk_force(r_idx, c_idx), res_exec_time(r_idx, c_idx));
    end
    fprintf('\n');
end

%% Print Benchmark Tables for All 6 Road Profiles
fprintf('========================================================================\n');
fprintf('  COMPREHENSIVE NUMERICAL BENCHMARK SUMMARY (ALL 36 RUNS)\n');
fprintf('========================================================================\n\n');

fprintf('TABLE I: RMS Body Acceleration [Ride Comfort] (m/s^2) [Lower is Better]\n');
fprintf('%-25s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n-------------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-25s', road_short{r});
    for c = 1:N_ctrl
        fprintf(' | %12.5f', res_rms_accel(r, c));
    end
    fprintf('\n');
end
fprintf('\n');

fprintf('TABLE II: RMS Suspension Deflection [Travel Stroke] (mm) [Lower is Better]\n');
fprintf('%-25s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n-------------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-25s', road_short{r});
    for c = 1:N_ctrl
        fprintf(' | %12.4f', res_rms_susp(r, c) * 1000);
    end
    fprintf('\n');
end
fprintf('\n');

fprintf('TABLE III: Peak Suspension Deflection [Rail Limit: 38.0 mm]\n');
fprintf('%-25s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n-------------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-25s', road_short{r});
    for c = 1:N_ctrl
        fprintf(' | %11.2f mm', res_pk_susp(r, c) * 1000);
    end
    fprintf('\n');
end
fprintf('\n');

fprintf('TABLE IV: Peak Actuator Control Force [Saturation Limit: +/- 39.2 N]\n');
fprintf('%-25s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n-------------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-25s', road_short{r});
    for c = 1:N_ctrl
        fprintf(' | %10.2f N', res_pk_force(r, c));
    end
    fprintf('\n');
end
fprintf('\n');

%% Export Results to CSV Dataset
csv_filename = fullfile(pwd, 'comparative_sweep_results.csv');
fid = fopen(csv_filename, 'w');
fprintf(fid, 'Road_Profile,RSI,Controller,RMS_Body_Accel_ms2,Peak_Body_Accel_ms2,RMS_Susp_Defl_mm,Peak_Susp_Defl_mm,RMS_Tire_Defl_mm,Peak_Tire_Defl_mm,Peak_Force_N,RMS_Force_N,Exec_Time_us\n');
for r = 1:N_roads
    for c = 1:N_ctrl
        fprintf(fid, '%s,%.2f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.2f\n', ...
            road_short{r}, rsi_vals(r), ctrl_short{c}, ...
            res_rms_accel(r, c), res_pk_accel(r, c), ...
            res_rms_susp(r, c)*1000, res_pk_susp(r, c)*1000, ...
            res_rms_tire(r, c)*1000, res_pk_tire(r, c)*1000, ...
            res_pk_force(r, c), res_rms_force(r, c), res_exec_time(r, c));
    end
end
fclose(fid);
fprintf('Benchmark dataset exported successfully to %s\n\n', csv_filename);

%% Generate High-Resolution Publication-Grade Plots (Figures 1 to 8)
fprintf('Generating IEEE publication-grade comparative figures for all 6 road profiles...\n');

colors = {
    [0.45, 0.45, 0.45], ... % Passive (Gray)
    [0.85, 0.33, 0.10], ... % Classical LQR (Red-Orange)
    [0.93, 0.69, 0.13], ... % ACO-LQR (Gold)
    [0.49, 0.18, 0.56], ... % HS-LQR (Purple)
    [0.00, 0.45, 0.74], ... % Fixed STSMC (Blue)
    [0.10, 0.65, 0.25]  ... % Adaptive STSMC (Emerald Green)
};
line_styles = {'-', '--', '-.', ':', '-', '-'};
line_widths = {1.2, 1.4, 1.4, 1.4, 1.8, 2.0};

for r = 1:N_roads
    fig = figure(r);
    clf(fig);
    set(fig, 'Position', [80, 80, 1100, 750], 'Color', 'w', 'PaperPositionMode', 'auto');
    
    % Subplot 1: Body Acceleration
    subplot(3, 1, 1);
    hold on;
    for c = [1, 2, 5, 6] % Passive, Classical LQR, Fixed STSMC, Adaptive STSMC
        traj = sim_trajectories{r, c};
        plot(traj.t, traj.zs_ddot, 'Color', colors{c}, 'LineStyle', line_styles{c}, ...
            'LineWidth', line_widths{c}, 'DisplayName', ctrl_names{c});
    end
    grid on; box on;
    ylabel('Body Accel (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
    title(sprintf('Comparative Dynamic Response: %s [RSI = %.2f]', road_names{r}, rsi_vals(r)), ...
        'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    xlim([0, min(8.0, T_stop)]);
    
    % Subplot 2: Suspension Deflection
    subplot(3, 1, 2);
    hold on;
    for c = [1, 2, 5, 6]
        traj = sim_trajectories{r, c};
        plot(traj.t, traj.x(1, :)*1000, 'Color', colors{c}, 'LineStyle', line_styles{c}, ...
            'LineWidth', line_widths{c}, 'DisplayName', ctrl_names{c});
    end
    yline(38, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Rail Limit (+38 mm)');
    yline(-38, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Rail Limit (-38 mm)');
    grid on; box on;
    ylabel('Susp Defl (mm)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    xlim([0, min(8.0, T_stop)]);
    
    % Subplot 3: Actuator Force
    subplot(3, 1, 3);
    hold on;
    for c = [2, 5, 6] % Active controllers only
        traj = sim_trajectories{r, c};
        plot(traj.t, traj.Fc, 'Color', colors{c}, 'LineStyle', line_styles{c}, ...
            'LineWidth', line_widths{c}, 'DisplayName', ctrl_names{c});
    end
    yline(39.2, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Saturation (+39.2 N)');
    yline(-39.2, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Saturation (-39.2 N)');
    grid on; box on;
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Force Fc (N)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    xlim([0, min(8.0, T_stop)]);
    
    fig_path = fullfile(fig_dir, sprintf('Figure%d_%s_Comparison.png', r, road_short{r}));
    exportgraphics(fig, fig_path, 'Resolution', 300);
    close(fig);
end

% Figure 7: Comprehensive Summary Performance Dashboard (All 6 Roads)
fig7 = figure(7);
clf(fig7);
set(fig7, 'Position', [100, 80, 1250, 700], 'Color', 'w', 'PaperPositionMode', 'auto');

subplot(1, 3, 1);
bar(res_rms_accel', 'grouped');
grid on; box on;
set(gca, 'XTickLabel', ctrl_short, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('RMS Body Accel (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title('Ride Comfort (RMS Accel)', 'FontSize', 11, 'FontWeight', 'bold');
legend(road_short, 'Location', 'northwest', 'FontSize', 7);

subplot(1, 3, 2);
bar((res_rms_susp * 1000)', 'grouped');
grid on; box on;
set(gca, 'XTickLabel', ctrl_short, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('RMS Susp Deflection (mm)', 'FontSize', 11, 'FontWeight', 'bold');
title('Working Space (RMS Stroke)', 'FontSize', 11, 'FontWeight', 'bold');

subplot(1, 3, 3);
bar(res_pk_force', 'grouped');
grid on; box on;
yline(39.2, 'r--', 'Limit (39.2 N)', 'LineWidth', 1.5);
set(gca, 'XTickLabel', ctrl_short, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('Peak Force (N)', 'FontSize', 11, 'FontWeight', 'bold');
title('Actuator Demand & Saturation', 'FontSize', 11, 'FontWeight', 'bold');

sgtitle('Quanser Active Suspension: 6-Road Profile Multi-Controller Benchmark', ...
    'FontSize', 14, 'FontWeight', 'bold');
fig7_path = fullfile(fig_dir, 'Figure7_Summary_Performance_Dashboard.png');
exportgraphics(fig7, fig7_path, 'Resolution', 300);
close(fig7);

% Figure 8: Suspension Deflection Deep Dive & Cumulative RMS Evolution
fig8 = figure(8);
clf(fig8);
set(fig8, 'Position', [120, 80, 1100, 750], 'Color', 'w', 'PaperPositionMode', 'auto');

r_deep = 3; % Severe Potholes
subplot(2, 1, 1);
hold on;
for c = [1, 2, 4, 5, 6]
    traj = sim_trajectories{r_deep, c};
    plot(traj.t, traj.x(1, :)*1000, 'Color', colors{c}, 'LineWidth', line_widths{c}, ...
        'DisplayName', ctrl_names{c});
end
yline(38, 'r--', 'Rail Stop (+38 mm)', 'LineWidth', 1.2);
yline(-38, 'r--', 'Rail Stop (-38 mm)', 'LineWidth', 1.2);
grid on; box on;
title('Deep Dive: Transient Suspension Deflection under Severe Potholes (RSI = 0.95)', ...
    'FontSize', 12, 'FontWeight', 'bold');
ylabel('Suspension Deflection (mm)', 'FontSize', 10, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0, 5.0]);

subplot(2, 1, 2);
hold on;
for c = [1, 2, 4, 5, 6]
    traj = sim_trajectories{r_deep, c};
    defl = traj.x(1, :)*1000;
    running_rms = sqrt(cumtrapz(traj.t, defl.^2) ./ max(traj.t, 1e-4));
    plot(traj.t, running_rms, 'Color', colors{c}, 'LineWidth', line_widths{c}, ...
        'DisplayName', sprintf('%s (Final: %.2f mm)', ctrl_short{c}, running_rms(end)));
end
grid on; box on;
title('Continuous Cumulative RMS Suspension Deflection Evolution Over Time', ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Cumulative RMS Defl (mm)', 'FontSize', 10, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);
xlim([0.1, 5.0]);

fig8_path = fullfile(fig_dir, 'Figure8_LQR_vs_STSMC_Deflection_DeepDive.png');
exportgraphics(fig8, fig8_path, 'Resolution', 300);
close(fig8);

fprintf('All 8 publication-grade figures saved to %s\n', fig_dir);
fprintf('========================================================================\n');
fprintf('  ALL 6 ROAD PROFILES FULLY TESTED AND BENCHMARKED WITH SUCCESS!\n');
fprintf('========================================================================\n\n');
