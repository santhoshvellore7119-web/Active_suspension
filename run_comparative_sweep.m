% RUN_COMPARATIVE_SWEEP
% Master Multi-Road & Multi-Controller Comparative Simulation Sweep
% Evaluating LQR (Classical, ACO, HS) vs. Super-Twisting Sliding Mode Control (STSMC)
%
% Compares 6 Controller Configurations across 4 RSI Road Profiles (24 Test Cases):
%   Controllers:
%     1. Passive Suspension (Zero Actuator Force Baseline)
%     2. Classical Quanser LQR (Standard Laboratory Feedback)
%     3. Optimal / ACO-Tuned LQR (Riccati ARE Optimal Design)
%     4. Harmony Search (HS) LQR (Dinesh Model Baseline)
%     5. Fixed STSMC (Moderate Fixed Bandwidth)
%     6. Adaptive STSMC (Road Severity Index RSI-Scheduled)
%
%   Road Profiles:
%     1. Smooth Road (RSI 0.10, 4 mm Sinusoid, 0.20 Hz)
%     2. Normal / Bumpy Road (RSI 0.35, 20 mm Sinusoid, 0.20 Hz)
%     3. Rough Road / ISO Class D (RSI 0.65, 15 mm Broadband Multi-Harmonic)
%     4. Pothole / Pulse Road (RSI 0.95, 20 mm Transient Impact Pulse, 0.30 Hz)

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  STARTING COMPREHENSIVE LQR VS. STSMC COMPARATIVE SIMULATION SWEEP\n');
fprintf('========================================================================\n\n');

setup_as;

dt = 0.001;
t_span = 0:dt:T_stop;
N_pts = length(t_span);

% Controller definitions
ctrl_names = {'Passive Baseline', 'Classical LQR', 'ACO-Tuned LQR', 'HS-Tuned LQR', 'Fixed STSMC', 'Adaptive STSMC'};
ctrl_short = {'Passive', 'Class-LQR', 'ACO-LQR', 'HS-LQR', 'Fixed-STSMC', 'Adap-STSMC'};
N_ctrl = length(ctrl_names);

% Road definitions
road_names = {'Smooth (RSI 0.10)', 'Normal (RSI 0.35)', 'Rough (RSI 0.65)', 'Pothole (RSI 0.95)'};
road_short = {'Smooth', 'Normal', 'Rough', 'Pothole'};
rsi_vals   = [0.10, 0.35, 0.65, 0.95];
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

% Pre-generate Road disturbance signals zr and zr_dot
road_zr     = cell(N_roads, 1);
road_zr_dot = cell(N_roads, 1);

% 1. Smooth Road: Sinusoid A=0.004 m, f=0.20 Hz
road_zr{1}     = road_smooth_amp * sin(2*pi*road_smooth_freq * t_span);
road_zr_dot{1} = road_smooth_amp * 2*pi*road_smooth_freq * cos(2*pi*road_smooth_freq * t_span);

% 2. Normal Road: Bumpy sinusoid A=0.020 m, f=0.20 Hz
road_zr{2}     = road_normal_amp * sin(2*pi*road_normal_freq * t_span);
road_zr_dot{2} = road_normal_amp * 2*pi*road_normal_freq * cos(2*pi*road_normal_freq * t_span);

% 3. Rough Road: ISO Class D multi-harmonic broadband profile
rng(42); % Fixed seed for exact reproducibility
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
road_zr{3}     = zr_rough;
road_zr_dot{3} = zr_dot_rough;

% 4. Pothole / Pulse Road: Square pulse 0.020 m, 0.30 Hz, 10% duty cycle
zr_pulse = zeros(1, N_pts);
zr_dot_pulse = zeros(1, N_pts);
period_pts = round((1/road_pothole_freq) / dt);
pulse_pts  = round(period_pts * (road_pothole_width / 100));

for i = 1:N_pts
    idx_in_period = mod(i-1, period_pts);
    if idx_in_period < pulse_pts
        zr_pulse(i) = road_pothole_amp;
    else
        zr_pulse(i) = 0.0;
    end
end
% Realistic tire rate-limited derivative
for i = 2:N_pts
    zr_dot_pulse(i) = (zr_pulse(i) - zr_pulse(i-1)) / dt;
    zr_dot_pulse(i) = max(min(zr_dot_pulse(i), 1.0), -1.0);
end
road_zr{4}     = zr_pulse;
road_zr_dot{4} = zr_dot_pulse;

% Output figures directory
fig_dir = fullfile(pwd, 'figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

fprintf('Executing 24-case benchmark (%d road profiles x %d controllers)...\n\n', N_roads, N_ctrl);

sim_trajectories = cell(N_roads, N_ctrl);

for r_idx = 1:N_roads
    fprintf('========================================================================\n');
    fprintf('  ROAD PROFILE %d: %s (RSI = %.2f)\n', r_idx, road_names{r_idx}, rsi_vals(r_idx));
    fprintf('========================================================================\n');
    zr_curr     = road_zr{r_idx};
    zr_dot_curr = road_zr_dot{r_idx};
    
    for c_idx = 1:N_ctrl
        tic;
        
        % State vector: x = [x1; x2; x3; x4] = [zs-zus; zs_dot; zus-zr; zus_dot]
        x       = zeros(4, N_pts);
        zs_ddot = zeros(1, N_pts);
        Fc      = zeros(1, N_pts);
        w_int   = 0.0;
        
        % Setup controller parameters
        if c_idx == 5 % Fixed STSMC
            c1_val = c1_fixed; k1_val = k1_fixed; k2_val = k2_fixed;
        elseif c_idx == 6 % Adaptive STSMC
            [k1_val, k2_val, c1_val] = stsmc_rsi_scheduler(r_idx, 'scaled');
        end
        
        % High-precision RK4 simulation loop
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
            
            % 4th-Order Runge-Kutta Integration Step
            k1_rk = x_dot;
            k2_rk = A * (x_k + 0.5*dt*k1_rk) + B * u_vec;
            k3_rk = A * (x_k + 0.5*dt*k2_rk) + B * u_vec;
            k4_rk = A * (x_k + dt*k3_rk) + B * u_vec;
            x(:, k+1) = x_k + (dt/6) * (k1_rk + 2*k2_rk + 2*k3_rk + k4_rk);
            
            % True sprung mass body acceleration:
            %   zs_ddot = (-ks*(zs-zus) - bs*(zs_dot-zus_dot) + u_sat) / ms
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
        res_exec_time(r_idx, c_idx) = (sim_time / N_pts) * 1e6; % microseconds per step
        
        sim_trajectories{r_idx, c_idx} = struct(...
            't', t_span, 'x', x, 'zs_ddot', zs_ddot, 'Fc', Fc, 'zr', zr_curr);
        
        % Safety check against Quanser physical limits
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

%% Print Comprehensive Publication-Grade Benchmark Tables
fprintf('========================================================================\n');
fprintf('  COMPREHENSIVE NUMERICAL BENCHMARK SUMMARY (ALL 24 RUNS)\n');
fprintf('========================================================================\n\n');

fprintf('TABLE I: RMS Body Acceleration [Ride Comfort] (m/s^2) [Lower is Better]\n');
fprintf('%-22s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-22s', road_names{r});
    for c = 1:N_ctrl
        fprintf(' | %12.5f', res_rms_accel(r, c));
    end
    fprintf('\n');
end
fprintf('\n');

fprintf('TABLE II: RMS Suspension Deflection [Travel Stroke] (mm) [Lower is Better]\n');
fprintf('%-22s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-22s', road_names{r});
    for c = 1:N_ctrl
        fprintf(' | %12.4f', res_rms_susp(r, c) * 1000);
    end
    fprintf('\n');
end
fprintf('\n');

fprintf('TABLE III: Peak Suspension Deflection [Mechanical Limit: 38.0 mm]\n');
fprintf('%-22s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-22s', road_names{r});
    for c = 1:N_ctrl
        fprintf(' | %11.2f mm', res_pk_susp(r, c) * 1000);
    end
    fprintf('\n');
end
fprintf('\n');

fprintf('TABLE IV: Peak Actuator Control Force [Saturation Limit: +/- 39.2 N]\n');
fprintf('%-22s', 'Road Profile');
for c = 1:N_ctrl, fprintf(' | %-12s', ctrl_short{c}); end
fprintf('\n------------------------------------------------------------------------------------------------------\n');
for r = 1:N_roads
    fprintf('%-22s', road_names{r});
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

%% Generate High-Resolution Publication-Grade Plots (Figures 1 to 6)
fprintf('Generating IEEE publication-grade comparative figures...\n');

% Color Palette for Controllers
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
    
    % Subplot 1: Sprung Mass Body Acceleration
    subplot(3, 1, 1);
    hold on;
    for c = [1, 2, 5, 6] % Focus on Passive, Classical LQR, Fixed STSMC, Adaptive STSMC
        traj = sim_trajectories{r, c};
        plot(traj.t, traj.zs_ddot, 'Color', colors{c}, 'LineStyle', line_styles{c}, ...
            'LineWidth', line_widths{c}, 'DisplayName', ctrl_names{c});
    end
    grid on; box on;
    ylabel('Body Accel (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
    title(sprintf('Comparative Dynamic Response: %s [RSI = %.2f]', road_names{r}, rsi_vals(r)), ...
        'FontSize', 13, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    xlim([0, min(6.0, T_stop)]);
    
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
    xlim([0, min(6.0, T_stop)]);
    
    % Subplot 3: Actuator Control Force
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
    xlim([0, min(6.0, T_stop)]);
    
    fig_path = fullfile(fig_dir, sprintf('Figure%d_%s_Comparison.png', r, road_short{r}));
    exportgraphics(fig, fig_path, 'Resolution', 300);
    close(fig);
end

% Figure 5: Comprehensive Summary Performance Dashboard (Bar Charts)
fig5 = figure(5);
clf(fig5);
set(fig5, 'Position', [100, 80, 1200, 700], 'Color', 'w', 'PaperPositionMode', 'auto');

% Subplot 1: RMS Body Acceleration
subplot(1, 3, 1);
b1 = bar(res_rms_accel', 'grouped');
grid on; box on;
set(gca, 'XTickLabel', ctrl_short, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('RMS Body Accel (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title('Ride Comfort Evaluation (RMS Accel)', 'FontSize', 11, 'FontWeight', 'bold');
legend(road_short, 'Location', 'northwest', 'FontSize', 8);

% Subplot 2: RMS Suspension Deflection
subplot(1, 3, 2);
b2 = bar((res_rms_susp * 1000)', 'grouped');
grid on; box on;
set(gca, 'XTickLabel', ctrl_short, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('RMS Susp Deflection (mm)', 'FontSize', 11, 'FontWeight', 'bold');
title('Suspension Working Space (RMS Stroke)', 'FontSize', 11, 'FontWeight', 'bold');

% Subplot 3: Peak Control Force Demand
subplot(1, 3, 3);
b3 = bar(res_pk_force', 'grouped');
grid on; box on;
yline(39.2, 'r--', 'Limit (39.2 N)', 'LineWidth', 1.5);
set(gca, 'XTickLabel', ctrl_short, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('Peak Force (N)', 'FontSize', 11, 'FontWeight', 'bold');
title('Actuator Demand & Saturation', 'FontSize', 11, 'FontWeight', 'bold');

sgtitle('Quanser Quarter-Car Active Suspension: Comprehensive Controller Benchmark', ...
    'FontSize', 14, 'FontWeight', 'bold');
fig5_path = fullfile(fig_dir, 'Figure5_Summary_Performance_Dashboard.png');
exportgraphics(fig5, fig5_path, 'Resolution', 300);
close(fig5);

% Figure 6: Detailed Suspension Deflection & RMS Evolution Deep Dive
fig6 = figure(6);
clf(fig6);
set(fig6, 'Position', [120, 80, 1100, 750], 'Color', 'w', 'PaperPositionMode', 'auto');

% Compare Pothole & Normal road deflection and running RMS
r_deep = 4; % Pothole shock
t_eval = sim_trajectories{r_deep, 1}.t;

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
title('Deep Dive: Transient Suspension Deflection under Severe Pothole Shock (RSI = 0.95)', ...
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

fig6_path = fullfile(fig_dir, 'Figure6_LQR_vs_STSMC_Deflection_DeepDive.png');
exportgraphics(fig6, fig6_path, 'Resolution', 300);
close(fig6);

fprintf('All 6 publication-grade figures saved to %s\n', fig_dir);
fprintf('========================================================================\n');
fprintf('  COMPARATIVE SWEEP FULLY COMPLETED WITH SUCCESS!\n');
fprintf('========================================================================\n\n');
