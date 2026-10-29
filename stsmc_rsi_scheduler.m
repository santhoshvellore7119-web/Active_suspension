function [k1, k2, c1, rsi_val, mode_name] = stsmc_rsi_scheduler(input_val, mode_type)
% STSMC_RSI_SCHEDULER Maps Road Severity Index (RSI) to Adaptive STSMC Controller Parameters
% Aligns exactly with the 4-Stage Methodology (VIT-EMPOWER GRANT 2026-27):
%   Stage 1: Road Perception (Edge AI: CNN -> RSI [0, 1])
%   Stage 2: Intelligent Decision Making (Adaptive Gain Scheduler: RSI -> k1, k2, lambda)
%   Stage 3: Adaptive Control (STSMC Controller -> u(t))
%   Stage 4: Performance Evaluation (Ride Comfort, Suspension Deflection, Dynamic Tire Load)
%
% Supports all 6 Road Profiles from Reference Repository (ananthu0987/Roadprofile_CSelec):
%   1: Harmonics Generator (Chirp / Multi-frequency Resonance)
%   2: Discrete Pulse Generator (Periodic Bump / Step Drop)
%   3: Continuous Signal Generator (Sinusoidal Road Undulation)
%   4: Smooth Road (ISO Class A Elevation Variations)
%   5: Normal Road (ISO Class B/C Rolling Undulations)
%   6: Pothole Road (Random Roughness + Severe Localized Dips)
%
% Inputs:
%   input_val : Road ID (1 to 6) OR Continuous RSI in [0, 1]
%   mode_type : 'scaled' (plant-optimized, default) or 'grant' (proposal exact table)
%
% Outputs:
%   k1, k2, c1: STSMC control parameters (c1 = lambda)
%   rsi_val   : Normalized Road Severity Index [0, 1]
%   mode_name : Descriptive string of operating mode / gain tier

if nargin < 2 || isempty(mode_type)
    mode_type = 'scaled';
end

% 1. Standard Discrete Road Profile Mappings (IDs 1 through 6)
if input_val >= 1 && input_val <= 6 && floor(input_val) == input_val
    road_idx = round(input_val);
    
    rsi_nodes_6 = [0.70, 0.85, 0.40, 0.10, 0.35, 0.95];
    
    if strcmpi(mode_type, 'grant')
        k1_nodes_6     = [18.0, 22.0, 13.0,  8.0, 12.0, 25.0];
        k2_nodes_6     = [ 7.0,  9.0,  4.5,  2.0,  4.0, 10.0];
        lambda_nodes_6 = [10.0, 12.5,  8.0,  5.0,  7.0, 15.0];
    else
        k1_nodes_6     = [130.0, 160.0, 70.0, 20.0, 60.0, 180.0];
        k2_nodes_6     = [ 55.0,  75.0, 30.0,  8.0, 25.0,  90.0];
        lambda_nodes_6 = [ 11.0,  13.5,  7.5,  5.0,  7.0,  15.0];
    end
    
    names_nodes_6 = { ...
        'Harmonics Chirp / Resonance (RSI = 0.70)', ...
        'Discrete Pulse Bump (RSI = 0.85)', ...
        'Signal Generator Sine Bump (RSI = 0.40)', ...
        'Smooth Road ISO Class A (RSI = 0.10)', ...
        'Normal Road ISO Class B/C (RSI = 0.35)', ...
        'Pothole Road Severe Dips (RSI = 0.95)' ...
    };

    rsi_val   = rsi_nodes_6(road_idx);
    k1        = k1_nodes_6(road_idx);
    k2        = k2_nodes_6(road_idx);
    c1        = lambda_nodes_6(road_idx);
    mode_name = names_nodes_6{road_idx};
    return;
end

% 2. Continuous RSI from Edge AI Perception (e.g. RSI in [0, 1])
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

rsi_val = max(0.0, min(1.0, input_val));
k1 = interp1(rsi_nodes, k1_nodes, rsi_val, 'linear', 'extrap');
k2 = interp1(rsi_nodes, k2_nodes, rsi_val, 'linear', 'extrap');
c1 = interp1(rsi_nodes, lambda_nodes, rsi_val, 'linear', 'extrap');

% Clamp to hardware safety limits
k1 = max(min(k1_nodes), min(max(k1_nodes), k1));
k2 = max(min(k2_nodes), min(max(k2_nodes), k2));
c1 = max(min(lambda_nodes), min(max(lambda_nodes), c1));

if rsi_val <= 0.25
    mode_name = sprintf('Smooth (Low Gain, RSI = %.2f)', rsi_val);
elseif rsi_val <= 0.50
    mode_name = sprintf('Normal (Medium Gain, RSI = %.2f)', rsi_val);
elseif rsi_val <= 0.80
    mode_name = sprintf('Rough (High Gain, RSI = %.2f)', rsi_val);
else
    mode_name = sprintf('Pothole (Max Gain, RSI = %.2f)', rsi_val);
end

end
