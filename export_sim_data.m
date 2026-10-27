% EXPORT_SIM_DATA
% Runs Master_Active_Suspension.slx across exact repository road profiles
% and saves time-domain trajectories and summary metrics to results/simulation_trajectories.mat
%
% Road Profiles (Exact Repository Definition):
%   1. Smooth Road (RSI 0.10): 4 mm Sine, 0.20 Hz
%   2. Normal Road (RSI 0.35): 20 mm Sine, 0.20 Hz
%   3. Rough Road (RSI 0.65) : ISO Class D Broadband (30 harmonics, rng 42)
%   4. Pothole Road (RSI 0.95): 20 mm Pulse Drops (0.30 Hz, 10% width)
%   5. Resonance Road        : 10 mm Sine at 2.58 Hz Body Hop Mode

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  RUNNING FULL SIMULINK SIMULATION SUITE FOR REPOSITORY ROAD PROFILES\n');
fprintf('========================================================================\n\n');

road_names = { ...
    'Smooth Road (RSI 0.10: 4 mm Sine, 0.20 Hz)', ...
    'Normal Road (RSI 0.35: 20 mm Sine, 0.20 Hz)', ...
    'Rough Road (RSI 0.65: ISO Class D Broadband, 90 km/h)', ...
    'Pothole Road (RSI 0.95: 20 mm Pulse Drops, 0.30 Hz)', ...
    'Harmonic Resonance (2.58 Hz Body Hop Mode)' ...
};
road_short = {'Smooth', 'Normal', 'Rough', 'Pothole', 'Resonance'};
rsi_labels = {'RSI 0.10', 'RSI 0.35', 'RSI 0.65', 'RSI 0.95', 'Resonance 2.58 Hz'};

N_roads = length(road_names);
sim_data = cell(N_roads, 1);

for r = 1:N_roads
    fprintf('Simulating Road Profile %d: %s...\n', r, road_names{r});
    res = simulate_case(r, false);
    res.name = road_names{r};
    res.short_name = road_short{r};
    res.rsi_label = rsi_labels{r};
    sim_data{r} = res;
end

if ~exist('results', 'dir'), mkdir('results'); end
save('results/simulation_trajectories.mat', 'sim_data', 'road_names', 'road_short', 'rsi_labels', '-v7');
fprintf('\nAll %d road profiles simulated and saved to results/simulation_trajectories.mat\n', N_roads);
