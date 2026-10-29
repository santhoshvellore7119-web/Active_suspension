% EXPORT_SIM_DATA
% Runs Master_Active_Suspension.slx across all 6 exact repository road profiles
% and saves time-domain trajectories and summary metrics to results/simulation_trajectories.mat
%
% 6 Standard Road Profiles from ananthu0987/Roadprofile_CSelec:
%   1. Harmonics Generator (Chirp 1-8 Hz, Resonant)
%   2. Pulse Generator (20 mm Discrete Bump)
%   3. Signal Generator (20 mm Rectified Sine Bump)
%   4. Smooth Road (5 mm Sine, Class A)
%   5. Normal Road (Dual Sine, Class B/C)
%   6. Pothole Road (Random Texture + Severe Localized Dips)

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  RUNNING FULL SIMULINK SIMULATION SUITE FOR ALL 6 ROAD PROFILES\n');
fprintf('========================================================================\n\n');

road_names = { ...
    'Harmonics Generator (Chirp 1-8 Hz, Resonant)', ...
    'Pulse Generator (20 mm Discrete Bump)', ...
    'Signal Generator (20 mm Rectified Sine)', ...
    'Smooth Road (5 mm Sine, Class A)', ...
    'Normal Road (Dual Sine, Class B/C)', ...
    'Pothole Road (Random Texture + Dips)' ...
};
road_short = {'Harmonics', 'Pulse', 'SignalGen_Sine', 'Smooth', 'Normal', 'Pothole'};
rsi_labels = {'Chirp 1-8 Hz (RSI 0.70)', 'Bump 20mm (RSI 0.85)', 'Sine Bump 20mm (RSI 0.40)', 'Class A (RSI 0.10)', 'Class B/C (RSI 0.35)', 'Potholes (RSI 0.95)'};

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
