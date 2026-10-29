function run_simulink_and_plots(road_id)
% RUN_SIMULINK_AND_PLOTS Opens Simulink model and displays plots in the same command
%
% Features:
%   - Opens Master_Active_Suspension.slx in the Simulink Editor GUI
%   - Simulates all 6 road profiles sequentially or an individual profile
%   - Opens a dedicated figure window for each road profile (Figure 1 to 6)
%   - Does NOT close previous figure windows, leaving all 6 windows open simultaneously!
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
%   run_simulink_and_plots          % Opens Simulink model and opens all 6 road profile plot windows (Figure 1 - 6)
%   run_simulink_and_plots('all')   % Opens Simulink model and opens all 6 road profile plot windows (Figure 1 - 6)
%   run_simulink_and_plots(1)       % Opens Simulink model and plots Road 1: Smooth road (Figure 1)
%   run_simulink_and_plots(2)       % Opens Simulink model and plots Road 2: Normal road (Figure 2)
%   run_simulink_and_plots(3)       % Opens Simulink model and plots Road 3: Rough road (Figure 3)
%   run_simulink_and_plots(4)       % Opens Simulink model and plots Road 4: Bumpy road (Figure 4)
%   run_simulink_and_plots(5)       % Opens Simulink model and plots Road 5: Pulse road (Figure 5)
%   run_simulink_and_plots(6)       % Opens Simulink model and plots Road 6: Potholes road (Figure 6)

    % Auto-align MATLAB directory to project folder
    proj_dir = fileparts(mfilename('fullpath'));
    if ~isempty(proj_dir) && ~strcmp(pwd, proj_dir)
        cd(proj_dir);
        addpath(genpath(proj_dir));
    end

    if nargin < 1 || isempty(road_id) || (ischar(road_id) && strcmpi(road_id, 'all'))
        run_roads = 1:6;
    elseif ischar(road_id)
        run_roads = str2double(road_id);
    else
        run_roads = road_id;
    end

    % 1. Determine model name without lock contention
    if bdIsLoaded('Master_Active_Suspension')
        model_name = 'Master_Active_Suspension';
    elseif bdIsLoaded('Master_Active_Suspension_Clean')
        model_name = 'Master_Active_Suspension_Clean';
    elseif exist('Master_Active_Suspension_Clean.slx', 'file')
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
    if usejava('desktop')
        open_system(model_name);
        fprintf('[1/2] Simulink model canvas opened in editor: %s.slx\n', model_name);
    else
        load_system(model_name);
        fprintf('[1/2] Simulink model loaded in memory: %s.slx\n', model_name);
    end

    % 3. Run simulations and render high-contrast comparison plots without closing previous ones
    road_names = { ...
        'Smooth road (4 mm, 0.2 Hz)', ...
        'Normal road (Resonance 2.58 Hz)', ...
        'Rough road (0.1-15 Hz Chirp)', ...
        'Bumpy road (Triangular Ramp)', ...
        'Pulse road (20 mm Step Drops)', ...
        'Potholes road (ISO Class D + Dips)' ...
    };

    fprintf('[2/2] Running simulations and opening plots for %d profile(s)...\n', length(run_roads));
    for idx = 1:length(run_roads)
        r = run_roads(idx);
        fprintf('  --> Simulating Profile %d: %s (Rendering Figure %d)...\n', r, road_names{r}, r);
        simulate_case(r, true);
        if usejava('desktop')
            drawnow;
        end
    end

    fprintf('\n========================================================================================\n');
    if length(run_roads) == 6
        fprintf('  SUCCESS: SIMULINK MODEL AND ALL 6 ROAD PROFILE PLOT WINDOWS OPENED SIMULTANEOUSLY!\n');
        fprintf('  - Figure 1: Smooth road\n');
        fprintf('  - Figure 2: Normal road (Resonance 2.58 Hz)\n');
        fprintf('  - Figure 3: Rough road (Chirp sweep)\n');
        fprintf('  - Figure 4: Bumpy road (Triangular ramp)\n');
        fprintf('  - Figure 5: Pulse road (20 mm step drops)\n');
        fprintf('  - Figure 6: Potholes road (ISO Class D + potholes)\n');
    else
        fprintf('  SUCCESS: SIMULINK MODEL AND FIGURE %d OPENED SIMULTANEOUSLY!\n', run_roads(1));
    end
    fprintf('========================================================================================\n');
end
