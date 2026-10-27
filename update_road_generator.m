% UPDATE_ROAD_GENERATOR.M
% Configures Master_Active_Suspension/Road_Surface_Generator
% to use the EXACT 4 road excitation profiles from the repository:
%   1. Smooth Road (RSI 0.10): 4 mm Sine, 0.20 Hz
%   2. Normal Road (RSI 0.35): 20 mm Sine, 0.20 Hz
%   3. Rough Road (RSI 0.65) : ISO Class D Broadband (30 harmonics, rng 42)
%   4. Pothole Road (RSI 0.95): 20 mm Pulse Drops (0.30 Hz, 10% width)
%   5. Resonance Road        : 10 mm Sine, 2.58 Hz Body Hop Mode

clear variables;
clc;

model_name = 'Master_Active_Suspension';
if ~bdIsLoaded(model_name), load_system(model_name); end
set_param(model_name, 'Lock', 'off');

road_sub = [model_name '/Road_Surface_Generator'];

% Initialize parameters in base workspace so blocks evaluate properly
parameters;

% 1. Reconfigure Profile 1: Smooth Sine
p1 = [road_sub '/Profile 1: Smooth Sine'];
if getSimulinkBlockHandle(p1) ~= -1
    set_param(p1, 'Amplitude', 'road_smooth_amp', 'Frequency', '2*pi*road_smooth_freq', 'SampleTime', '0');
end

% 2. Reconfigure Profile 2: Normal Sine (was Pothole)
% Delete old Profile 2 block
p2_old = [road_sub '/Profile 2: Pothole Pulse'];
p2_new = [road_sub '/Profile 2: Normal Sine'];
if getSimulinkBlockHandle(p2_old) ~= -1
    pos2 = get_param(p2_old, 'Position');
    ph2 = get_param(p2_old, 'PortHandles');
    l2 = get_param(ph2.Outport, 'Line');
    if l2 ~= -1, delete_line(l2); end
    delete_block(p2_old);
    
    add_block('simulink/Sources/Sine Wave', p2_new, ...
        'Amplitude', 'road_normal_amp', 'Frequency', '2*pi*road_normal_freq', ...
        'SampleTime', '0', 'Position', pos2);
    add_line(road_sub, 'Profile 2: Normal Sine/1', 'Mux_Switch/3');
end

% 3. Reconfigure Profile 3: Rough Road (ISO Class D From Workspace)
% Delete old Profile 3 Chirp & Gain
p3_chirp = [road_sub '/Profile 3: Chirp'];
p3_gain  = [road_sub '/Gain_Chirp_Amp'];
p3_new   = [road_sub '/Profile 3: Rough ISO Class D'];
if getSimulinkBlockHandle(p3_chirp) ~= -1
    pos3 = get_param(p3_chirp, 'Position');
    if getSimulinkBlockHandle(p3_gain) ~= -1
        ph_g = get_param(p3_gain, 'PortHandles');
        lg = get_param(ph_g.Outport, 'Line');
        if lg ~= -1, delete_line(lg); end
        delete_block(p3_gain);
    end
    delete_block(p3_chirp);
    
    add_block('simulink/Sources/From Workspace', p3_new, ...
        'VariableName', 'road_rough_data', 'SampleTime', '0', 'Interpolate', 'on', ...
        'Position', [pos3(1), pos3(2), pos3(1)+120, pos3(4)]);
    add_line(road_sub, 'Profile 3: Rough ISO Class D/1', 'Mux_Switch/4');
end

% 4. Reconfigure Profile 4: Pothole Pulse (was ISO Class D Sine)
p4_old = [road_sub '/Profile 4: ISO Class D'];
p4_new = [road_sub '/Profile 4: Pothole Pulse'];
if getSimulinkBlockHandle(p4_old) ~= -1
    pos4 = get_param(p4_old, 'Position');
    ph4 = get_param(p4_old, 'PortHandles');
    l4 = get_param(ph4.Outport, 'Line');
    if l4 ~= -1, delete_line(l4); end
    delete_block(p4_old);
    
    add_block('simulink/Sources/Pulse Generator', p4_new, ...
        'PulseType', 'Time based', ...
        'Amplitude', 'road_pothole_amp', ...
        'Period', '1/road_pothole_freq', ...
        'PulseWidth', 'road_pothole_width', ...
        'PhaseDelay', '0.0', ...
        'Position', pos4);
    add_line(road_sub, 'Profile 4: Pothole Pulse/1', 'Mux_Switch/5');
end

% 5. Reconfigure Profile 5: Resonance (2.58 Hz)
p5 = [road_sub '/Profile 5: Resonance (2.58 Hz)'];
if getSimulinkBlockHandle(p5) ~= -1
    set_param(p5, 'Amplitude', 'road5_amp', 'Frequency', '2*pi*road5_freq', 'SampleTime', '0');
end

% Save and confirm
save_system(model_name);
fprintf('Successfully updated Master_Active_Suspension/Road_Surface_Generator with exact repository profiles!\n');
