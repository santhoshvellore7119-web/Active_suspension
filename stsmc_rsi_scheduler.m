function [k1, k2, c1, rsi_val, mode_name] = stsmc_rsi_scheduler(input_val, mode_type)
% STSMC_RSI_SCHEDULER Maps Road Severity Index (RSI) to Adaptive STSMC Controller Parameters
% Supports all 6 Standardized Road Profiles from the Automotive Active Suspension Benchmark Suite:
%   Profile 1: Dynamic Frequency-Sweep (Chirp: 0.1 - 10 Hz)         [RSI = 0.50]
%   Profile 2: Smooth Undulating Highway (4 mm Sine, 0.20 Hz)        [RSI = 0.10]
%   Profile 3: Severe Discrete Potholes (20 mm Transient Drops)      [RSI = 0.95]
%   Profile 4: Triangular Ramp & Obstacle / Speed Bump               [RSI = 0.40]
%   Profile 5: Real-World Stochastic Rough Terrain + Pothole Shocks  [RSI = 0.85]
%   Profile 6: Harmonic Corrugated Resonance Road (f ~ 2.58 Hz)     [RSI = 0.65]
%
% Inputs:
%   input_val : Discrete Road Profile ID (1 to 6) OR Continuous RSI in [0, 1]
%   mode_type : 'scaled' (plant-optimized, default) or 'grant' (exact proposal table)
%
% Outputs:
%   k1, k2, c1: STSMC control parameters (c1 = sliding surface slope lambda)
%   rsi_val   : Normalized Road Severity Index in [0, 1]
%   mode_name : Descriptive string of operating mode / road benchmark

if nargin < 2 || isempty(mode_type)
    mode_type = 'scaled';
end

% Standard Reference RSI Breakpoints for continuous interpolation
rsi_nodes   = [0.10, 0.35, 0.65, 0.95];

if strcmpi(mode_type, 'grant')
    k1_nodes     = [8.0, 12.0, 18.0, 25.0];
    k2_nodes     = [2.0,  4.0,  7.0, 10.0];
    lambda_nodes = [5.0,  7.0, 10.0, 15.0];
else
    k1_nodes     = [20.0, 60.0, 120.0, 180.0];
    k2_nodes     = [8.0,  25.0,  50.0,  90.0];
    lambda_nodes = [5.0,   7.0,  10.0,  15.0];
end

% Parse Input: Discrete Road Profile ID (1..6) or Continuous RSI [0..1]
if input_val >= 1 && input_val <= 6 && floor(input_val) == input_val
    road_id = round(input_val);
    switch road_id
        case 1 % Frequency-Sweep Chirp (Broadband frequency exploration)
            rsi_val   = 0.50;
            c1        = 8.5;
            k1        = 85.0;
            k2        = 38.0;
            mode_name = 'Frequency-Sweep Chirp (Bandwidth Mode)';
            
        case 2 % Smooth Undulating Highway (Comfort-oriented low gains)
            rsi_val   = 0.10;
            c1        = 5.0;
            k1        = 20.0;
            k2        = 8.0;
            mode_name = 'Smooth Undulating Road (Comfort Mode)';
            
        case 3 % Severe Discrete Potholes (Max disturbance rejection & deadbeat settling)
            rsi_val   = 0.95;
            c1        = 15.0;
            k1        = 180.0;
            k2        = 90.0;
            mode_name = 'Severe Pothole Impacts (Max Disturbance Rejection)';
            
        case 4 % Triangular Ramp & Speed Bump (Stroke absorption & smooth ride height return)
            rsi_val   = 0.40;
            c1        = 7.5;
            k1        = 70.0;
            k2        = 30.0;
            mode_name = 'Triangular Ramp / Speed Bump (Stroke Mode)';
            
        case 5 % Real-World Stochastic Rough + Heavy Pothole Shocks (Simultaneous broadband & impact)
            rsi_val   = 0.85;
            c1        = 12.5;
            k1        = 150.0;
            k2        = 70.0;
            mode_name = 'Stochastic Rough + Shocks (Dual-Regime Mode)';
            
        case 6 % Harmonic Resonance Road (Exciting quarter-car body resonance at ~2.58 Hz)
            rsi_val   = 0.65;
            c1        = 10.0;
            k1        = 120.0;
            k2        = 50.0;
            mode_name = 'Harmonic Corrugated Resonance Road (Anti-Resonance Mode)';
    end
else
    % Continuous RSI input from Edge AI CNN Perception (0.00 <= rsi_val <= 1.00)
    rsi_val = max(0.0, min(1.0, input_val));
    k1 = interp1(rsi_nodes, k1_nodes, rsi_val, 'linear', 'extrap');
    k2 = interp1(rsi_nodes, k2_nodes, rsi_val, 'linear', 'extrap');
    c1 = interp1(rsi_nodes, lambda_nodes, rsi_val, 'linear', 'extrap');
    
    % Enforce strict physical safety bounds
    k1 = max(min(k1_nodes), min(max(k1_nodes), k1));
    k2 = max(min(k2_nodes), min(max(k2_nodes), k2));
    c1 = max(min(lambda_nodes), min(max(lambda_nodes), c1));
    
    if rsi_val <= 0.25
        mode_name = sprintf('Smooth Highway Comfort (RSI = %.2f)', rsi_val);
    elseif rsi_val <= 0.50
        mode_name = sprintf('Normal Urban Driving (RSI = %.2f)', rsi_val);
    elseif rsi_val <= 0.75
        mode_name = sprintf('Rough Terrain / Resonance (RSI = %.2f)', rsi_val);
    else
        mode_name = sprintf('Severe Obstacle Shock (RSI = %.2f)', rsi_val);
    end
end

end
