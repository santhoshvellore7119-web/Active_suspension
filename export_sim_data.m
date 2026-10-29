% EXPORT_SIM_DATA
% Runs Master_Active_Suspension.slx across all 6 exact professor road profiles
% and saves time-domain trajectories and summary metrics to results/simulation_trajectories.mat
%
% 6 Standard Road Profiles from Professor Specifications:
%   1. Smooth road   (4 mm, 0.20 Hz Sine Wave)
%   2. Normal road   (Harmonic Near Body Resonance 2.58 Hz)
%   3. Rough road    (0.1 - 15.0 Hz Frequency Chirp Sweep)
%   4. Bumpy road    (Triangular Ramp Obstacle)
%   5. Pulse road    (20 mm Step Pulse Drops)
%   6. Potholes road (Broadband ISO Class D + Deep Potholes)

clear variables;
clc;

fprintf('========================================================================\n');
fprintf('  RUNNING FULL SIMULINK SIMULATION SUITE FOR ALL 6 ROAD PROFILES\n');
fprintf('========================================================================\n\n');

road_names = { ...
    'Smooth road', ...
    'Normal road', ...
    'Rough road', ...
    'Bumpy road', ...
    'Pulse road', ...
    'Potholes road' ...
};
road_short = {'Smooth road', 'Normal road', 'Rough road', 'Bumpy road', 'Pulse road', 'Potholes road'};
rsi_labels = {'4mm Sine (RSI 0.10)', 'Resonance 2.58Hz (RSI 0.65)', '0.1-15Hz Chirp (RSI 0.75)', 'Triangular Ramp (RSI 0.40)', '20mm Pulse (RSI 0.85)', 'Deep Potholes (RSI 0.95)'};

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
