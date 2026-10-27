function fig = plot_comparisons(res, road_title)
% PLOT_COMPARISONS Generates publication-ready comparative response plots
% Comparing ONLY the 3 ACTIVE CONTROLLERS:
%   (1) Optimal LQR (Linear Quadratic Regulator)
%   (2) Fixed STSMC (Super-Twisting Sliding Mode Control)
%   (3) Adaptive STSMC (Online Dynamic Gain Adaptation)
%
% Features:
%   - Exactly 3 active controllers (no passive baseline)
%   - Steady-state RMS & peak metrics displayed in legends
%   - Distinct line styles and line widths (no hidden traces)
%   - Dynamic y-axis auto-zooming with headroom (no squashing signals)
%   - Non-colliding limit compliance badges
%   - Clean pure white background with pitch-black crisp labels and axes

    if nargin < 2 || isempty(road_title)
        road_title = 'Quarter-Car Comparative Benchmark';
    end

    t  = res.time(:);
    t_end = t(end);
    t_settle = t_end * 0.50;
    if isfield(res, 't_settle'), t_settle = res.t_settle; end

    % Set figure properties
    fig = figure(201);
    clf(fig);
    set(fig, 'Name', sprintf('Active Suspension Dynamic Response: %s', road_title), ...
             'Color', 'w', 'NumberTitle', 'off', 'Position', [80, 50, 1150, 850], ...
             'PaperPositionMode', 'auto');

    % Define high-contrast distinct color palette & line styles
    % LQR:       Classic Navy blue, dashed, 1.8 pt
    % Fixed SMC: Bright Amber/Crimson, dash-dot, 1.8 pt
    % Adap SMC:  Emerald Green, solid, 1.8 pt
    c_lqr  = [0.12, 0.47, 0.71];
    c_fsmc = [1.00, 0.50, 0.05];
    c_asmc = [0.17, 0.63, 0.17];

    % --- Subplot 1: Sprung Mass Body Vertical Acceleration (Ride Comfort) ---
    ax1 = subplot(3, 1, 1);
    hold on;
    xline(t_settle, ':', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    
    p1 = plot(t, res.lqr.acc,  'Color', c_lqr,  'LineStyle', '--', 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Optimal LQR (SS RMS: %.3f m/s^2 | Pk: %.3f)', res.lqr.rms_acc_ss, res.lqr.pk_acc));
    p2 = plot(t, res.fsmc.acc, 'Color', c_fsmc, 'LineStyle', '-.', 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Fixed STSMC (SS RMS: %.3f m/s^2 | Pk: %.3f)', res.fsmc.rms_acc_ss, res.fsmc.pk_acc));
    p3 = plot(t, res.asmc.acc, 'Color', c_asmc, 'LineStyle', '-',  'LineWidth', 1.8, ...
        'DisplayName', sprintf('Adaptive STSMC (SS RMS: %.3f m/s^2 | Pk: %.3f)', res.asmc.rms_acc_ss, res.asmc.pk_acc));
    hold off;
    grid on; box on;
    set(ax1, 'Color', 'w', 'FontSize', 9.5, 'XColor', 'k', 'YColor', 'k', ...
             'GridLineStyle', ':', 'GridColor', [0.7 0.7 0.7], 'GridAlpha', 0.6, ...
             'XTickLabel', []);
    ylabel('Body Accel (m/s^2)', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', 'k');
    title(sprintf('Active Suspension Dynamic Response: %s', road_title), ...
          'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    xlim([0, t_end]);
    acc_max = max([abs(res.lqr.acc); abs(res.fsmc.acc); abs(res.asmc.acc)]);
    if acc_max < 0.05, acc_max = 0.05; end
    ylim([-acc_max * 1.30, acc_max * 1.30]);
    legend([p1, p2, p3], 'Location', 'northeast', 'FontSize', 9.0, 'NumColumns', 3, ...
           'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);

    % --- Subplot 2: Relative Suspension Deflection / Travel Stroke ---
    ax2 = subplot(3, 1, 2);
    hold on;
    xline(t_settle, ':', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(t, res.lqr.def,  'Color', c_lqr,  'LineStyle', '--', 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Optimal LQR (Peak: %.1f mm)', res.lqr.pk_def));
    plot(t, res.fsmc.def, 'Color', c_fsmc, 'LineStyle', '-.', 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Fixed STSMC (Peak: %.1f mm)', res.fsmc.pk_def));
    plot(t, res.asmc.def, 'Color', c_asmc, 'LineStyle', '-',  'LineWidth', 1.8, ...
        'DisplayName', sprintf('Adaptive STSMC (Peak: %.1f mm)', res.asmc.pk_def));
    
    def_max = max([abs(res.lqr.def); abs(res.fsmc.def); abs(res.asmc.def)]);
    if def_max < 0.5, def_max = 1.0; end

    % Dynamic Zooming vs. Limit Lines
    if def_max < 25.0
        y_lim_def = max(def_max * 1.35, 1.0);
        ylim([-y_lim_def, y_lim_def]);
        text(0.02*t_end, 0.85*y_lim_def, ...
            sprintf('Rail Travel Limit: +/- 38.0 mm  |  Max Active: %.1f mm  [PASS: SAFE]', def_max), ...
            'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.1 0.45 0.9], ...
            'BackgroundColor', [0.94 0.97 1.0], 'EdgeColor', [0.6 0.75 0.95], 'Margin', 3, 'Interpreter', 'none');
    else
        y_lim_def = max(def_max * 1.15, 42.0);
        ylim([-y_lim_def, y_lim_def]);
        yline(38.0, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Travel Limit (+38 mm)');
        yline(-38.0, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end

    hold off;
    grid on; box on;
    set(ax2, 'Color', 'w', 'FontSize', 9.5, 'XColor', 'k', 'YColor', 'k', ...
             'GridLineStyle', ':', 'GridColor', [0.7 0.7 0.7], 'GridAlpha', 0.6, ...
             'XTickLabel', []);
    ylabel('Susp. Travel (mm)', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', 'k');
    xlim([0, t_end]);
    legend('Location', 'southeast', 'FontSize', 9.0, 'NumColumns', 3, ...
           'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);

    % --- Subplot 3: Actuator Control Force Demands ---
    ax3 = subplot(3, 1, 3);
    hold on;
    xline(t_settle, ':', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(t, res.lqr.u,  'Color', c_lqr,  'LineStyle', '--', 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Optimal LQR (Peak: %.1f N)', res.lqr.pk_u));
    plot(t, res.fsmc.u, 'Color', c_fsmc, 'LineStyle', '-.', 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Fixed STSMC (Peak: %.1f N)', res.fsmc.pk_u));
    plot(t, res.asmc.u, 'Color', c_asmc, 'LineStyle', '-',  'LineWidth', 1.8, ...
        'DisplayName', sprintf('Adaptive STSMC (Peak: %.1f N)', res.asmc.pk_u));
    
    u_max = max([abs(res.lqr.u); abs(res.fsmc.u); abs(res.asmc.u)]);
    if u_max < 1.0, u_max = 2.0; end

    % Dynamic Zooming vs. Limit Lines
    if u_max < 30.0
        y_lim_u = max(u_max * 1.35, 1.5);
        ylim([-y_lim_u, y_lim_u]);
        text(0.02*t_end, 0.85*y_lim_u, ...
            sprintf('Motor Saturation Limit: +/- 39.2 N  |  Peak Commanded: %.1f N  [PASS: SAFE]', u_max), ...
            'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.12 0.5 0.2], ...
            'BackgroundColor', [0.95 0.98 0.95], 'EdgeColor', [0.6 0.8 0.6], 'Margin', 3, 'Interpreter', 'none');
    else
        y_lim_u = max(u_max * 1.15, 44.0);
        ylim([-y_lim_u, y_lim_u]);
        yline(39.2, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Saturation (+39.2 N)');
        yline(-39.2, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end

    hold off;
    grid on; box on;
    set(ax3, 'Color', 'w', 'FontSize', 9.5, 'XColor', 'k', 'YColor', 'k', ...
             'GridLineStyle', ':', 'GridColor', [0.7 0.7 0.7], 'GridAlpha', 0.6);
    xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', 'k');
    ylabel('Control Force (N)', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', 'k');
    xlim([0, t_end]);
    legend('Location', 'southeast', 'FontSize', 9.0, 'NumColumns', 3, ...
           'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);

    linkaxes([ax1, ax2, ax3], 'x');

    if ~exist('results', 'dir'), mkdir('results'); end
    saveas(fig, fullfile('results', 'last_simulation_response.png'));
end
