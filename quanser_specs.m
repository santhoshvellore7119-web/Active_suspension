% QUANSER_SPECS
% Technical Specifications and Physical Constraints Lookup
% Quanser Active Suspension Experiment (ASE)

function specs = quanser_specs()
    specs = struct();
    
    % Mechanical specs
    specs.ms  = 2.45;     % Sprung mass (kg)
    specs.mu  = 1.00;     % Unsprung mass (kg)
    specs.ks  = 900.0;    % Suspension stiffness (N/m)
    specs.kt  = 2500.0;   % Tire stiffness (N/m)
    specs.bs  = 7.5;      % Suspension damping (N-s/m)
    specs.bus = 5.0;      % Tire damping (N-s/m)
    
    % Physical travel limits (User Manual Table 3.1)
    specs.St_suspension_travel_range = 0.038; % 38 mm total allowable stroke
    specs.St_middle_plate_travel     = 0.035; % 35 mm
    specs.St_bottom_plate_travel     = 0.036; % 36 mm
    specs.St_max_deflection          = 0.038; % Peak allowable suspension deflection (m)
    
    % Actuator limits (Faulhaber / Capstan DC motor)
    specs.Fc_max = 39.2;  % Maximum continuous control force (N)
    specs.Fc_min = -39.2; % Minimum control force (N)
    specs.Sm     = 0.112; % Torque constant (N-m/A)
    
    % Sensors
    specs.Se_suspension_encoder = 0.000942; % Resolution (m/count)
    specs.As_accelerometer_sens = 9.81;     % Sensitivity (m/s^2/V)
    
    if nargout == 0
        fprintf('\n=== Quanser Active Suspension Technical Specifications ===\n');
        fprintf('Sprung Mass (ms):                  %.2f kg\n', specs.ms);
        fprintf('Unsprung Mass (mu):                %.2f kg\n', specs.mu);
        fprintf('Suspension Stiffness (ks):         %.1f N/m\n', specs.ks);
        fprintf('Tire Stiffness (kt):               %.1f N/m\n', specs.kt);
        fprintf('Suspension Travel Range Limit:     %.3f m (38 mm)\n', specs.St_suspension_travel_range);
        fprintf('Actuator Continuous Force Limit:   +/- %.1f N\n', specs.Fc_max);
        fprintf('===========================================================\n\n');
    end
end
