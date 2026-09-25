% AUTO_PLOT_RESULTS
% Automatic Post-Simulation Plotting Function
% Invoked automatically by Simulink StopFcn callback or called manually.

function auto_plot_results()
    % Check if simulation data exists in workspace
    vars_exist = evalin('base', 'exist(''zs_ddot_sim'', ''var'') && exist(''susp_defl_sim'', ''var'')');
    if ~vars_exist
        return;
    end
    
    try
        % Extract logged signals from base workspace
        zs_ddot   = evalin('base', 'zs_ddot_sim');
        susp_defl = evalin('base', 'susp_defl_sim');
        Fc_force  = evalin('base', 'Fc_force_sim');
        
        if evalin('base', 'exist(''tout'', ''var'')')
            t = evalin('base', 'tout');
        else
            t = linspace(0, 10, length(zs_ddot))';
        end
        
        if evalin('base', 'exist(''zr_sim'', ''var'')')
            zr = evalin('base', 'zr_sim');
        else
            zr = zeros(size(t));
        end
        
        if evalin('base', 'exist(''zs_sim'', ''var'')')
            zs = evalin('base', 'zs_sim');
        else
            zs = susp_defl;
        end
        
        if evalin('base', 'exist(''zus_sim'', ''var'')')
            zus = evalin('base', 'zus_sim');
        else
            zus = zeros(size(t));
        end
        
        % Ensure column vectors
        t = t(:);
        zs_ddot = zs_ddot(:);
        susp_defl = susp_defl(:);
        Fc_force = Fc_force(:);
        zr = zr(:);
        zs = zs(:);
        zus = zus(:);
        
        % Check controller and road labels
        ctrl_id = 6;
        if evalin('base', 'exist(''CONTROLLER_ID'', ''var'')')
            ctrl_id = evalin('base', 'CONTROLLER_ID');
        end
        ctrl_names = {'Passive', 'Classical LQR', 'ACO-LQR', 'HS-LQR', 'Fixed STSMC', 'Adaptive STSMC'};
        ctrl_str = ctrl_names{max(1, min(6, ctrl_id))};
        
        road_id = 2;
        if evalin('base', 'exist(''ROAD_PROFILE_ID'', ''var'')')
            road_id = evalin('base', 'ROAD_PROFILE_ID');
        end
        road_names = {'Smooth (RSI 0.10)', 'Normal (RSI 0.35)', 'Rough (RSI 0.65)', 'Pothole (RSI 0.95)'};
        road_str = road_names{max(1, min(4, road_id))};
        
        % Create Interactive Figure Window
        fig = figure(101);
        clf(fig);
        set(fig, 'Name', sprintf('Quanser Active Suspension Response - %s [%s]', ctrl_str, road_str), ...
                 'NumberTitle', 'off', 'Color', 'w', 'Position', [100, 80, 1100, 850]);
        
        % 1. Displacements: zr, zus, zs
        subplot(4, 1, 1);
        plot(t, zr * 1000, 'k--', 'LineWidth', 1.2); hold on;
        plot(t, zus * 1000, 'b-', 'LineWidth', 1.3);
        plot(t, zs * 1000, 'r-', 'LineWidth', 1.8);
        grid on; box on;
        ylabel('Displacement (mm)', 'FontWeight', 'bold');
        title(sprintf('Quanser Active Suspension System Response | Controller: %s | Road: %s', ctrl_str, road_str), ...
              'FontSize', 12, 'FontWeight', 'bold');
        legend('Road Elevation z_r', 'Wheel Displacement z_{us}', 'Body Displacement z_s', 'Location', 'best');
        xlim([0, t(end)]);
        
        % 2. Vehicle Body Acceleration
        subplot(4, 1, 2);
        plot(t, zs_ddot, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5);
        grid on; box on;
        ylabel('Body Accel (m/s^2)', 'FontWeight', 'bold');
        title(sprintf('Body Acceleration [Ride Comfort] (RMS: %.4f m/s^2, Peak: %.4f m/s^2)', ...
              sqrt(mean(zs_ddot.^2)), max(abs(zs_ddot))), 'FontWeight', 'bold');
        xlim([0, t(end)]);
        
        % 3. Suspension Deflection
        subplot(4, 1, 3);
        plot(t, susp_defl * 1000, 'Color', [0.0 0.5 0.0], 'LineWidth', 1.5); hold on;
        yline(38, 'r--', 'Upper Rail Limit (+38 mm)', 'LineWidth', 1.2);
        yline(-38, 'r--', 'Lower Rail Limit (-38 mm)', 'LineWidth', 1.2);
        grid on; box on;
        ylabel('Susp Defl (mm)', 'FontWeight', 'bold');
        title(sprintf('Suspension Deflection (z_s - z_{us}) [Peak: %.2f mm / 38 mm Limit]', max(abs(susp_defl))*1000), ...
              'FontWeight', 'bold');
        ylim([-45, 45]);
        xlim([0, t(end)]);
        
        % 4. Actuator Control Force
        subplot(4, 1, 4);
        plot(t, Fc_force, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.5); hold on;
        yline(39.2, 'r--', 'Actuator Max (+39.2 N)', 'LineWidth', 1.2);
        yline(-39.2, 'r--', 'Actuator Min (-39.2 N)', 'LineWidth', 1.2);
        grid on; box on;
        xlabel('Time (seconds)', 'FontWeight', 'bold');
        ylabel('Force F_c (N)', 'FontWeight', 'bold');
        title(sprintf('Actuator Force F_c [Peak: %.1f N / 39.2 N Saturation]', max(abs(Fc_force))), 'FontWeight', 'bold');
        ylim([-45, 45]);
        xlim([0, t(end)]);
        
        drawnow;
    catch ME
        fprintf('Auto plot notice: %s\n', ME.message);
    end
end
