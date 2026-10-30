function [k1, k2, c1, rsi_val, mode_name] = stsmc_rsi_scheduler(input_val, mode_type)
% STSMC_RSI_SCHEDULER Maps Road Severity Index (RSI) to Adaptive STSMC Controller Parameters
% Aligns exactly with Professor Specifications (4-Stage Methodology, VIT-EMPOWER GRANT 2026-27):
%   Stage 1: Road Perception (Edge AI: CNN -> RSI [0, 1])
%   Stage 2: Intelligent Decision Making (Adaptive Gain Scheduler: RSI -> k1, k2, lambda)
%   Stage 3: Adaptive Control (STSMC Controller -> u(t))
%   Stage 4: Performance Evaluation (Ride Comfort, Suspension Deflection, Dynamic Tire Load)
%
% 6 Standard Road Profiles from Professor Specifications:
%   1: Smooth road   (4 mm, 0.20 Hz Sine Wave)
%   2: Normal road   (Harmonic Near Body Resonance 2.58 Hz)
%   3: Rough road    (0.1 - 15.0 Hz Frequency Chirp Sweep)
%   4: Bumpy road    (Triangular Ramp Obstacle)
%   5: Pulse road    (20 mm Step Pulse Drops)
%   6: Potholes road (Broadband ISO Class D + Deep Potholes)
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
is_continuous_mode = strcmpi(mode_type, 'continuous') || strcmpi(mode_type, 'rsi');
if ~is_continuous_mode && input_val >= 1 && input_val <= 6 && floor(input_val) == input_val
    road_idx = round(input_val);
    
    rsi_nodes_6 = [0.10, 0.65, 0.75, 0.40, 0.85, 0.95];
    
    if strcmpi(mode_type, 'grant')
        k1_nodes_6     = [ 8.0, 18.0, 20.0, 13.0, 22.0, 25.0];
        k2_nodes_6     = [ 2.0,  7.0,  8.0,  4.5,  9.0, 10.0];
        lambda_nodes_6 = [ 5.0, 10.0, 12.0,  8.0, 13.5, 15.0];
    else
        % Precision-tuned for optimal ride comfort while preserving hardware travel safety
        k1_nodes_6     = [ 20.0,  55.0,  62.0, 45.0,  75.0,  78.0];
        k2_nodes_6     = [  8.0,  22.0,  25.0, 18.0,  30.0,  32.0];
        lambda_nodes_6 = [  5.0,   6.5,   6.8,  5.8,   7.8,   8.0];
    end
    
    names_nodes_6 = { ...
        'Smooth road', ...
        'Normal road', ...
        'Rough road', ...
        'Bumpy road', ...
        'Pulse road', ...
        'Potholes road' ...
    };

    rsi_val   = rsi_nodes_6(road_idx);
    k1        = k1_nodes_6(road_idx);
    k2        = k2_nodes_6(road_idx);
    c1        = lambda_nodes_6(road_idx);
    mode_name = names_nodes_6{road_idx};
    return;
end

% 2. Continuous RSI from Edge AI Perception (e.g. RSI in [0, 1])
% Comprehensive 8-node monotonic spectrum covering all future arbitrary road profile geometries:
rsi_nodes = [0.00, 0.10, 0.40, 0.65, 0.75, 0.85, 0.95, 1.00];
if strcmpi(mode_type, 'grant')
    k1_nodes     = [ 6.0,  8.0, 13.0, 18.0, 20.0, 22.0, 25.0, 26.0];
    k2_nodes     = [ 1.5,  2.0,  4.5,  7.0,  8.0,  9.0, 10.0, 10.5];
    lambda_nodes = [ 4.5,  5.0,  8.0, 10.0, 12.0, 13.5, 15.0, 15.5];
else
    % Continuous monotonic gain scheduling (smooth comfort to robust rejection)
    k1_nodes     = [15.0, 20.0, 45.0, 55.0, 62.0, 75.0, 78.0, 85.0];
    k2_nodes     = [ 6.0,  8.0, 18.0, 22.0, 25.0, 30.0, 32.0, 35.0];
    lambda_nodes = [ 4.5,  5.0,  5.8,  6.5,  6.8,  7.8,  8.0,  8.5];
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
    mode_name = sprintf('Smooth road (Low Gain, RSI = %.2f)', rsi_val);
elseif rsi_val <= 0.50
    mode_name = sprintf('Bumpy road (Medium Gain, RSI = %.2f)', rsi_val);
elseif rsi_val <= 0.80
    mode_name = sprintf('Normal / Rough road (High Gain, RSI = %.2f)', rsi_val);
else
    mode_name = sprintf('Potholes road (Max Disturbance Rejection, RSI = %.2f)', rsi_val);
end

end
