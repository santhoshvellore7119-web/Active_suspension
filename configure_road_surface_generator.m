% CONFIGURE_ROAD_SURFACE_GENERATOR
% Reconfigures Master_Active_Suspension_Simulation/Road Surface Generator
%
% Implements:
%   1. A clean selector where the user enters a number (1 to 6) to select the desired road
%   2. A Multiport Switch ("Mux") selecting between 6 road profiles:
%        Input 1: Original Quanser Harmonics Generator (already present)
%        Input 2: Smooth Undulating Highway (4 mm Sine, 0.20 Hz)
%        Input 3: Severe Discrete Potholes (20 mm Step Drops)
%        Input 4: Triangular Ramp Obstacle & Speed Bump (20 mm Wedge)
%        Input 5: Real-World Stochastic Rough Terrain + Shocks (15 mm)
%        Input 6: Harmonic Corrugated Resonance Road (2.58 Hz Body Mode)
%   3. Connects cleanly to Outport zr while preserving the exact modular Quanser layout.

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  CONFIGURING 6 ROAD PROFILES & NUMBER SELECTOR IN SIMULINK MODEL\n');
fprintf('========================================================================\n\n');

setup_as;

model_name = 'Master_Active_Suspension_Simulation';
road_sub   = [model_name '/Road Surface Generator'];

if ~bdIsLoaded(model_name)
    load_system(model_name);
end
set_param(model_name, 'Lock', 'off');

%% 1. Clean up old legacy manual switches in Road Surface Generator
blks_to_del = { ...
    [road_sub '/Manual Switch'], ...
    [road_sub '/Manual Switch1'], ...
    [road_sub '/Constant'], ...
    [road_sub '/Signal Generator'], ...
    [road_sub '/Pulse\nGenerator'], ...
    [road_sub '/Pulse Generator'], ...
    [road_sub '/Saturation'], ...
    [road_sub '/Scope'], ...
    [road_sub '/Scope1'], ...
    [road_sub '/Scope2'] ...
};

for i = 1:length(blks_to_del)
    b = blks_to_del{i};
    if getSimulinkBlockHandle(b) ~= -1
        ph = get_param(b, 'PortHandles');
        all_ports = [ph.Inport, ph.Outport];
        for p = all_ports
            l = get_param(p, 'Line');
            if l ~= -1, delete_line(l); end
        end
        delete_block(b);
    end
end

% Delete existing line into Outport zr if any
zr_out = [road_sub '/zr'];
if getSimulinkBlockHandle(zr_out) ~= -1
    zr_ph = get_param(zr_out, 'PortHandles');
    l = get_param(zr_ph.Inport(1), 'Line');
    if l ~= -1, delete_line(l); end
end

%% 2. Add Road Profile Selector Constant block inside Road Surface Generator
selector_blk = [road_sub '/Enter Road Number (1 to 6)'];
if getSimulinkBlockHandle(selector_blk) == -1
    add_block('simulink/Sources/Constant', selector_blk, ...
        'Value', 'ROAD_PROFILE_ID', ...
        'Position', [320, 20, 420, 50]);
else
    set_param(selector_blk, 'Value', 'ROAD_PROFILE_ID', 'Position', [320, 20, 420, 50]);
end

%% 3. Add the 6 Road Profile Source Blocks inside Road Surface Generator

% Profile 1: Original Quanser Harmonics Generator (Already present at [200, 45, 240, 105])
harm_blk = [road_sub '/ Harmonics Generator'];
if getSimulinkBlockHandle(harm_blk) ~= -1
    set_param(harm_blk, 'Position', [120, 80, 220, 130]);
end

% Profile 2: Smooth Undulating Highway (4 mm Sine, 0.20 Hz)
p2_blk = [road_sub '/Profile 2: Smooth Highway Sine (4 mm, 0.2 Hz)'];
if getSimulinkBlockHandle(p2_blk) == -1
    add_block('simulink/Sources/Sine Wave', p2_blk, ...
        'Amplitude', 'road_smooth_amp', 'Frequency', '2*pi*road_smooth_freq', 'SampleTime', '0', ...
        'Position', [120, 160, 220, 200]);
end

% Profile 3: Severe Discrete Potholes (20 mm Transient Pulse Drop)
p3_blk = [road_sub '/Profile 3: Severe Pothole Cavities (-20 mm)'];
if getSimulinkBlockHandle(p3_blk) == -1
    add_block('simulink/Sources/Pulse Generator', p3_blk, ...
        'Amplitude', '-road_pothole_amp', 'Period', '1/road_pothole_freq', ...
        'PulseWidth', 'road_pothole_width', 'PhaseDelay', '0.2', ...
        'Position', [120, 230, 220, 270]);
end

% Profile 4: Triangular Ramp Obstacle & Speed Bump (20 mm Wedge)
p4_blk = [road_sub '/Profile 4: Triangular Ramp & Speed Bump (+20 mm)'];
if getSimulinkBlockHandle(p4_blk) == -1
    add_block('simulink/Sources/Repeating Sequence', p4_blk, ...
        'rep_seq_t', '[0, 0.5, 1.5, 1.52, 4.0]', 'rep_seq_y', '[0, 0.020, 0.020, 0, 0]', ...
        'Position', [120, 300, 220, 340]);
end

% Profile 5: Real-World Stochastic Rough Terrain + Shocks (15 mm)
p5_blk = [road_sub '/Profile 5: Stochastic Rough Terrain + Shocks'];
if getSimulinkBlockHandle(p5_blk) == -1
    add_block('simulink/Sources/Sine Wave', p5_blk, ...
        'Amplitude', 'road_rough_amp', 'Frequency', '2*pi*4.5', 'SampleTime', '0', ...
        'Position', [120, 370, 220, 410]);
end

% Profile 6: Harmonic Corrugated Resonance Road (10 mm, 2.58 Hz Body Mode)
p6_blk = [road_sub '/Profile 6: Harmonic Resonance Corrugation (2.58 Hz)'];
if getSimulinkBlockHandle(p6_blk) == -1
    add_block('simulink/Sources/Sine Wave', p6_blk, ...
        'Amplitude', 'road_res_amp', 'Frequency', '2*pi*road_res_freq', 'SampleTime', '0', ...
        'Position', [120, 440, 220, 480]);
end

%% 4. Add Multiport Switch (Mux) to select between all 6 profiles
mux_blk = [road_sub '/Road_Profile_Mux_Switch'];
if getSimulinkBlockHandle(mux_blk) == -1
    add_block('simulink/Signal Routing/Multiport Switch', mux_blk, ...
        'Inputs', '6', ...
        'DataPortOrder', 'One-based contiguous', ...
        'Position', [480, 50, 520, 510]);
else
    % Clear any existing lines on the switch ports to prevent duplicate connection errors
    mph = get_param(mux_blk, 'PortHandles');
    for p = [mph.Inport, mph.Outport]
        l = get_param(p, 'Line');
        if l ~= -1
            try
                delete_line(l);
            catch
            end
        end
    end
end

% Wire control input: Enter Road Number -> Multiport Switch Control Port (port 1)
add_line(road_sub, 'Enter Road Number (1 to 6)/1', 'Road_Profile_Mux_Switch/1', 'autorouting', 'on');

% Wire 6 data inputs:
% Port 2: Profile 1 (Harmonics Generator)
add_line(road_sub, ' Harmonics Generator/1', 'Road_Profile_Mux_Switch/2', 'autorouting', 'on');

% Port 3: Profile 2 (Smooth Highway Sine)
add_line(road_sub, 'Profile 2: Smooth Highway Sine (4 mm, 0.2 Hz)/1', 'Road_Profile_Mux_Switch/3', 'autorouting', 'on');

% Port 4: Profile 3 (Severe Potholes)
add_line(road_sub, 'Profile 3: Severe Pothole Cavities (-20 mm)/1', 'Road_Profile_Mux_Switch/4', 'autorouting', 'on');

% Port 5: Profile 4 (Triangular Ramp)
add_line(road_sub, 'Profile 4: Triangular Ramp & Speed Bump (+20 mm)/1', 'Road_Profile_Mux_Switch/5', 'autorouting', 'on');

% Port 6: Profile 5 (Stochastic Rough + Shocks)
add_line(road_sub, 'Profile 5: Stochastic Rough Terrain + Shocks/1', 'Road_Profile_Mux_Switch/6', 'autorouting', 'on');

% Port 7: Profile 6 (Resonance Corrugation)
add_line(road_sub, 'Profile 6: Harmonic Resonance Corrugation (2.58 Hz)/1', 'Road_Profile_Mux_Switch/7', 'autorouting', 'on');

% Wire Multiport Switch output -> Outport zr
set_param(zr_out, 'Position', [620, 270, 650, 290]);
add_line(road_sub, 'Road_Profile_Mux_Switch/1', 'zr/1', 'autorouting', 'on');

%% 5. Keep root canvas clean (delete any unconnected selector on root)
root_selector = [model_name '/Enter Road Number (1 to 6)'];
if getSimulinkBlockHandle(root_selector) ~= -1
    delete_block(root_selector);
end

save_system(model_name, fullfile(pwd, [model_name '.slx']));
close_system(model_name, 0);

fprintf('========================================================================\n');
fprintf('  6 ROAD PROFILES & MULTIPORT SWITCH (MUX) CONFIGURED SUCCESSFULLY!\n');
fprintf('  Enter any number from 1 to 6 to immediately simulate that road.\n');
fprintf('========================================================================\n\n');
