% RUN_COMPARATIVE_SWEEP
% Pure STSMC Multi-Road & Multi-Tier Comparative Simulation Sweep
%
% Compares Super-Twisting Sliding Mode Control configurations across 4 RSI Road Profiles:
%   Configurations:
%     1. Passive Suspension (Zero Control Baseline)
%     2. STSMC Conservative (c1=5, k1=20, k2=8)
%     3. STSMC Moderate     (c1=7, k1=60, k2=25)
%     4. STSMC Aggressive   (c1=15, k1=180, k2=90)
%     5. RSI-Adaptive STSMC (Intelligent Scheduled Gains)
%
%   Road Profiles:
%     1. Smooth Road (RSI 0.10)
%     2. Normal / Bumpy Road (RSI 0.35)
%     3. Rough Road / ISO Class D (RSI 0.65)
%     4. Pothole / Pulse Road (RSI 0.95)

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  STARTING PURE STSMC ACTIVE SUSPENSION COMPARATIVE SWEEP\n');
fprintf('========================================================================\n\n');

setup_as;

dt = 0.001;
t_span = 0:dt:T_stop;
N_pts = length(t_span);

% Controller definitions
ctrl_names = {'Passive', 'STSMC Conservative', 'STSMC Moderate', 'STSMC Aggressive', 'Adaptive STSMC'};
N_ctrl = length(ctrl_names);

% Road definitions
road_names = {'Smooth (RSI 0.10)', 'Normal (RSI 0.35)', 'Rough (RSI 0.65)', 'Pothole (RSI 0.95)'};
rsi_vals   = [0.10, 0.35, 0.65, 0.95];
N_roads    = length(road_names);

% Storage arrays for results
res_rms_accel = zeros(N_roads, N_ctrl);
res_pk_accel  = zeros(N_roads, N_ctrl);
res_rms_susp  = zeros(N_roads, N_ctrl);
res_pk_susp   = zeros(N_roads, N_ctrl);
res_rms_tire  = zeros(N_roads, N_ctrl);
res_pk_force  = zeros(N_roads, N_ctrl);
res_rms_force = zeros(N_roads, N_ctrl);
res_exec_time = zeros(N_roads, N_ctrl);

% Pre-generate Road disturbance signals zr and zr_dot
road_zr     = cell(N_roads, 1);
road_zr_dot = cell(N_roads, 1);

% 1. Smooth Road: Sinusoid A=0.004m, f=0.2Hz
road_zr{1}     = road_smooth_amp * sin(2*pi*road_smooth_freq * t_span);
road_zr_dot{1} = road_smooth_amp * 2*pi*road_smooth_freq * cos(2*pi*road_smooth_freq * t_span);

% 2. Normal Road: Bumpy sinusoid A=0.02m, f=0.2Hz
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
    zr_rough = zr_rough + amp_harm(k) * sin(omega_harm(k)*t_span + phi_harm(k));
    zr_dot_rough = zr_dot_rough + amp_harm(k) * omega_harm(k) * cos(omega_harm(k)*t_span + phi_harm(k));
end
road_zr{3}     = zr_rough;
road_zr_dot{3} = zr_dot_rough;

% 4. Pothole / Pulse Road: Square pulse 0.02m, 0.3Hz, 10% duty cycle
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
% Filtered derivative
for i = 2:N_pts
    zr_dot_pulse(i) = (zr_pulse(i) - zr_pulse(i-1)) / dt;
    zr_dot_pulse(i) = max(min(zr_dot_pulse(i), 1.0), -1.0); % Realistic tire ramp rate
end
road_zr{4}     = zr_pulse;
road_zr_dot{4} = zr_dot_pulse;

% Setup output figures directory
fig_dir = fullfile(pwd, 'figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

fprintf('Running pure STSMC simulation sweep across %d road conditions x %d configurations = %d runs...\n\n', ...
    N_roads, N_ctrl, N_roads*N_ctrl);

sim_trajectories = cell(N_roads, N_ctrl);

for r_idx = 1:N_roads
    fprintf('--- Simulating Road Profile %d: %s ---\n', r_idx, road_names{r_idx});
    zr_curr     = road_zr{r_idx};
    zr_dot_curr = road_zr_dot{r_idx};
    
    for c_idx = 1:N_ctrl
        tic;
        
        % State: x = [x1; x2; x3; x4] = [zs-zus; zs_dot; zus-zr; zus_dot]
        x = zeros(4, N_pts);
        zs_ddot = zeros(1, N_pts);
        Fc      = zeros(1, N_pts);
        w_int   = 0.0;
        
        % Determine gains for current controller configuration
        if c_idx == 1 % Passive
            c1_val = 0; k1_val = 0; k2_val = 0;
        elseif c_idx == 2 % STSMC Conservative
            c1_val = 5.0; k1_val = 20.0; k2_val = 8.0;
        elseif c_idx == 3 % STSMC Moderate
            c1_val = 7.0; k1_val = 60.0; k2_val = 25.0;
        elseif c_idx == 4 % STSMC Aggressive
            c1_val = 15.0; k1_val = 180.0; k2_val = 90.0;
        else % Adaptive STSMC
            [k1_val, k2_val, c1_val] = stsmc_rsi_scheduler(r_idx, 'scaled');
        end
        
        % Run discrete time simulation
        for k = 1:N_pts-1
            x_k = x(:, k);
            zr_dot_k = zr_dot_curr(k);
            
            % Compute Control Force
            if c_idx == 1
                u = 0.0;
            else
                s = c1_val * x_k(1) + x_k(2);
                tanh_s = tanh(s / eps_bl);
                u1 = -k1_val * sqrt(abs(s)) * tanh_s;
                w_int = w_int - k2_val * tanh_s * dt;
                w_int = max(min(w_int, Fc_max), Fc_min);
                u = u1 + w_int;
            end
            
            % Apply physical actuator saturation (+/- 39.2 N)
            u_sat = max(min(u, Fc_max), Fc_min);
            Fc(k) = u_sat;
            
            % Compute state derivative: x_dot = A*x + B*[zr_dot; u_sat]
            u_vec = [zr_dot_k; u_sat];
            x_dot = A * x_k + B * u_vec;
            
            % Integrate (RK4 step)
            k1_rk = x_dot;
            k2_rk = A * (x_k + 0.5*dt*k1_rk) + B * u_vec;
            k3_rk = A * (x_k + 0.5*dt*k2_rk) + B * u_vec;
            k4_rk = A * (x_k + dt*k3_rk) + B * u_vec;
            x(:, k+1) = x_k + (dt/6) * (k1_rk + 2*k2_rk + 2*k3_rk + k4_rk);
            
            % Acceleration: zs_ddot = -ks/ms*(zs-zus) - bs/ms*(zs_dot-zus_dot) + u_sat/ms
            zs_ddot(k) = (-ks * x_k(1) - bs * (x_k(2) - x_k(4)) + u_sat) / ms;
        end
        zs_ddot(N_pts) = zs_ddot(N_pts-1);
        Fc(N_pts) = Fc(N_pts-1);
        
        sim_time = toc;
        
        % Record Performance Metrics
        res_rms_accel(r_idx, c_idx) = rms(zs_ddot);
        res_pk_accel(r_idx, c_idx)  = max(abs(zs_ddot));
        res_rms_susp(r_idx, c_idx)  = rms(x(1, :));
        res_pk_susp(r_idx, c_idx)   = max(abs(x(1, :)));
        res_rms_tire(r_idx, c_idx)  = rms(x(3, :));
        res_pk_force(r_idx, c_idx)  = max(abs(Fc));
        res_rms_force(r_idx, c_idx) = rms(Fc);
        res_exec_time(r_idx, c_idx) = sim_time;
        
        sim_trajectories{r_idx, c_idx} = struct('t', t_span, 'x', x, 'zs_ddot', zs_ddot, 'Fc', Fc, 'zr', zr_curr);
        
        fprintf('  [%-18s] RMS Accel: %7.4f m/s^2 | RMS Susp: %7.5f m (Pk: %6.4f m) | Pk Force: %5.1f N | Time: %5.3f s\n', ...
            ctrl_names{c_idx}, res_rms_accel(r_idx, c_idx), res_rms_susp(r_idx, c_idx), ...
            res_pk_susp(r_idx, c_idx), res_pk_force(r_idx, c_idx), sim_time);
    end
    fprintf('\n');
end

%% Print Comprehensive Summary Tables
fprintf('========================================================================\n');
fprintf('  COMPREHENSIVE PURE STSMC RESULTS SUMMARY (ALL ROADS & MODES)\n');
fprintf('========================================================================\n\n');

fprintf('TABLE I: RMS Body Acceleration [Ride Comfort] (m/s^2)\n');
fprintf('%-22s', 'Road Condition');
for c = 1:N_ctrl, fprintf('| %-18s', ctrl_names{c}); end
fprintf('\n%s\n', repmat('-', 1, 22 + N_ctrl*21));
for r = 1:N_roads
    fprintf('%-22s', road_names{r});
    for c = 1:N_ctrl
        fprintf('| %18.5f', res_rms_accel(r, c));
    end
    fprintf('\n');
end

fprintf('\nTABLE II: RMS Suspension Deflection [Suspension Travel] (m)\n');
fprintf('%-22s', 'Road Condition');
for c = 1:N_ctrl, fprintf('| %-18s', ctrl_names{c}); end
fprintf('\n%s\n', repmat('-', 1, 22 + N_ctrl*21));
for r = 1:N_roads
    fprintf('%-22s', road_names{r});
    for c = 1:N_ctrl
        fprintf('| %18.6f', res_rms_susp(r, c));
    end
    fprintf('\n');
end

fprintf('\nTABLE III: Peak Control Force [Actuator Effort <= 39.2 N] (N)\n');
fprintf('%-22s', 'Road Condition');
for c = 1:N_ctrl, fprintf('| %-18s', ctrl_names{c}); end
fprintf('\n%s\n', repmat('-', 1, 22 + N_ctrl*21));
for r = 1:N_roads
    fprintf('%-22s', road_names{r});
    for c = 1:N_ctrl
        fprintf('| %18.2f', res_pk_force(r, c));
    end
    fprintf('\n');
end

%% Generate & Save High-Resolution Figures
fprintf('\nGenerating high-resolution comparative figures...\n');

colors = [0.2 0.2 0.2; 0.0 0.447 0.741; 0.85 0.325 0.098; 0.929 0.694 0.125; 0.466 0.674 0.188];
line_styles = {'-', '--', '-.', ':', '-'};

for r = 1:N_roads
    f = figure('Position', [100, 100, 1100, 850], 'Visible', 'off');
    
    % Subplot 1: Road & Body Displacement
    subplot(4, 1, 1); hold on; grid on; box on;
    plot(t_span, road_zr{r}*1000, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Road (zr)');
    for c = 1:N_ctrl
        plot(t_span, (sim_trajectories{r,c}.x(1,:) + road_zr{r})*1000, ...
            'Color', colors(c,:), 'LineStyle', line_styles{c}, 'LineWidth', 1.4, 'DisplayName', ctrl_names{c});
    end
    ylabel('Position (mm)'); title(sprintf('Road Profile %d: %s - Pure STSMC Verification', r, road_names{r}), 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'eastoutside', 'FontSize', 8);
    xlim([0 10]);
    
    % Subplot 2: Suspension Deflection
    subplot(4, 1, 2); hold on; grid on; box on;
    yline(38, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Travel Limit (+38 mm)');
    yline(-38, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    for c = 1:N_ctrl
        plot(t_span, sim_trajectories{r,c}.x(1,:)*1000, ...
            'Color', colors(c,:), 'LineStyle', line_styles{c}, 'LineWidth', 1.4, 'DisplayName', ctrl_names{c});
    end
    ylabel('Deflection (mm)'); ylim([-45 45]);
    legend('Location', 'eastoutside', 'FontSize', 8);
    xlim([0 10]);
    
    % Subplot 3: Sprung Mass Acceleration
    subplot(4, 1, 3); hold on; grid on; box on;
    for c = 1:N_ctrl
        plot(t_span, sim_trajectories{r,c}.zs_ddot, ...
            'Color', colors(c,:), 'LineStyle', line_styles{c}, 'LineWidth', 1.4, 'DisplayName', ctrl_names{c});
    end
    ylabel('Accel (m/s^2)');
    legend('Location', 'eastoutside', 'FontSize', 8);
    xlim([0 10]);
    
    % Subplot 4: Actuator Force
    subplot(4, 1, 4); hold on; grid on; box on;
    yline(39.2, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Limit (+39.2 N)');
    yline(-39.2, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    for c = 1:N_ctrl
        plot(t_span, sim_trajectories{r,c}.Fc, ...
            'Color', colors(c,:), 'LineStyle', line_styles{c}, 'LineWidth', 1.4, 'DisplayName', ctrl_names{c});
    end
    xlabel('Time (s)'); ylabel('Force (N)'); ylim([-45 45]);
    legend('Location', 'eastoutside', 'FontSize', 8);
    xlim([0 10]);
    
    fig_name = sprintf('Figure%d_%s_Comparison.png', r, strtok(road_names{r}, ' '));
    saveas(f, fullfile(fig_dir, fig_name));
    close(f);
end

% Summary Dashboard
f_dash = figure('Position', [150, 150, 1100, 650], 'Visible', 'off');

subplot(1, 2, 1);
b1 = bar(res_rms_accel); grid on;
set(gca, 'XTickLabel', {'Smooth', 'Normal', 'Rough', 'Pothole'});
ylabel('RMS Sprung Mass Acceleration (m/s^2)');
title('Ride Comfort Comparison (Lower is Better)', 'FontSize', 11, 'FontWeight', 'bold');
legend(ctrl_names, 'Location', 'northwest', 'FontSize', 8);

subplot(1, 2, 2);
b2 = bar(res_pk_force); grid on;
yline(39.2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Force Limit (+39.2 N)');
set(gca, 'XTickLabel', {'Smooth', 'Normal', 'Rough', 'Pothole'});
ylabel('Peak Actuator Force (N)');
title('Actuator Control Effort (Limit = 39.2 N)', 'FontSize', 11, 'FontWeight', 'bold');
legend(ctrl_names, 'Location', 'northwest', 'FontSize', 8);

saveas(f_dash, fullfile(fig_dir, 'Figure5_Summary_Performance_Dashboard.png'));
close(f_dash);

%% Export CSV Summary Table
csv_file = fullfile(pwd, 'comparative_sweep_results.csv');
fid = fopen(csv_file, 'w');
fprintf(fid, 'Road_Profile,RSI,Controller,RMS_Acceleration_m_s2,Peak_Acceleration_m_s2,RMS_Suspension_Deflection_m,Peak_Suspension_Deflection_m,RMS_Tire_Deflection_m,Peak_Control_Force_N,RMS_Control_Force_N,Execution_Time_s\n');
for r = 1:N_roads
    for c = 1:N_ctrl
        fprintf(fid, '%s,%.2f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
            road_names{r}, rsi_vals(r), ctrl_names{c}, ...
            res_rms_accel(r, c), res_pk_accel(r, c), ...
            res_rms_susp(r, c), res_pk_susp(r, c), ...
            res_rms_tire(r, c), res_pk_force(r, c), ...
            res_rms_force(r, c), res_exec_time(r, c));
    end
end
fclose(fid);
fprintf('Full numerical dataset exported to: %s\n\n', csv_file);

fprintf('========================================================================\n');
fprintf('  PURE STSMC COMPARATIVE SWEEP COMPLETED SUCCESSFULLY\n');
fprintf('========================================================================\n');
