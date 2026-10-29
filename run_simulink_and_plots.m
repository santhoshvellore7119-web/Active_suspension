function run_simulink_and_plots(road_id)
% RUN_SIMULINK_AND_PLOTS Opens Simulink model and displays plots in the same command
% 
% Features:
%   - Opens Master_Active_Suspension.slx in the Simulink Editor
%   - Executes parallel 3-controller simulation (Optimal LQR vs Fixed STSMC vs Adaptive STSMC)
%   - Renders 3-subplot high-contrast dynamic response figure in the same command
%
% 6 Standard Road Profiles from Professor Specifications:
%   1 : Smooth road   (4 mm, 0.20 Hz Sine Wave)
%   2 : Normal road   (Harmonic Near Body Resonance 2.58 Hz)
%   3 : Rough road    (0.1 - 15.0 Hz Frequency Chirp Sweep)
%   4 : Bumpy road    (Triangular Ramp Obstacle)
%   5 : Pulse road    (20 mm Step Pulse Drops)
%   6 : Potholes road (Broadband ISO Class D + Deep Potholes)
%
% Usage:
%   run_simulink_and_plots       % Runs Road 1: Smooth road
%   run_simulink_and_plots(1)    % Runs Road 1: Smooth road
%   run_simulink_and_plots(2)    % Runs Road 2: Normal road
%   run_simulink_and_plots(3)    % Runs Road 3: Rough road
%   run_simulink_and_plots(4)    % Runs Road 4: Bumpy road
%   run_simulink_and_plots(5)    % Runs Road 5: Pulse road
%   run_simulink_and_plots(6)    % Runs Road 6: Potholes road

    if nargin < 1 || isempty(road_id)
        road_id = 1; % Default: Smooth road
    end

    % 1. Determine model name
    if exist('Master_Active_Suspension_Clean.slx', 'file')
        try
            copyfile('Master_Active_Suspension_Clean.slx', 'Master_Active_Suspension.slx');
            model_name = 'Master_Active_Suspension';
        catch
            model_name = 'Master_Active_Suspension_Clean';
        end
    else
        model_name = 'Master_Active_Suspension';
    end

    fprintf('========================================================================================\n');
    fprintf('  OPENING SIMULINK CANVAS & EXECUTING DYNAMIC BENCHMARK (SAME COMMAND)\n');
    fprintf('========================================================================================\n\n');

    % 2. Open Simulink model architecture in editor
    open_system(model_name);
    fprintf('[1/2] Simulink model canvas opened in editor: %s.slx\n', model_name);

    % 3. Run simulation and render high-contrast comparison plot
    fprintf('[2/2] Running simulation and rendering comparison plots for Road %d...\n', road_id);
    simulate_case(road_id, true);

    fprintf('========================================================================================\n');
    fprintf('  SUCCESS: SIMULINK CANVAS AND PLOTS OPENED SIMULTANEOUSLY!\n');
    fprintf('========================================================================================\n');
end
