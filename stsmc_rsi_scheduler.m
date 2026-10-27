function [k1, k2, c1, rsi_val, mode_name] = stsmc_rsi_scheduler(input_val, mode_type)
% STSMC_RSI_SCHEDULER Maps Road Severity Index (RSI) to Adaptive STSMC Controller Parameters
% Aligns exactly with the 4-Stage Methodology (VIT-EMPOWER GRANT 2026-27):
%   Stage 1: Road Perception (Edge AI: CNN -> RSI [0, 1])
%   Stage 2: Intelligent Decision Making (Adaptive Gain Scheduler: RSI -> k1, k2, lambda)
%   Stage 3: Adaptive Control (STSMC Controller -> u(t))
%   Stage 4: Performance Evaluation (Ride Comfort, Suspension Deflection, Dynamic Tire Load)
%
% Inputs:
%   input_val : Road ID (1=Smooth, 2=Normal, 3=Rough, 4=Pothole) OR Continuous RSI in [0, 1]
%   mode_type : 'scaled' (plant-optimized, default) or 'grant' (proposal exact table)
%
% Outputs:
%   k1, k2, c1: STSMC control parameters (c1 = lambda)
%   rsi_val   : Normalized Road Severity Index [0, 1]
%   mode_name : Descriptive string of operating mode / gain tier

if nargin < 2 || isempty(mode_type)
    mode_type = 'scaled';
end

% 1. Reference RSI Breakpoints
rsi_nodes   = [0.10, 0.35, 0.65, 0.95];
names_nodes = {'Smooth (Low Gain)', 'Normal (Medium Gain)', 'Rough (High Gain)', 'Pothole (Max Disturbance Rejection)'};

if strcmpi(mode_type, 'grant')
    k1_nodes     = [8.0, 12.0, 18.0, 25.0];
    k2_nodes     = [2.0,  4.0,  7.0, 10.0];
    lambda_nodes = [5.0,  7.0, 10.0, 15.0];
else
    k1_nodes     = [20.0, 60.0, 120.0, 180.0];
    k2_nodes     = [8.0,  25.0,  50.0,  90.0];
    lambda_nodes = [5.0,   7.0,  10.0,  15.0];
end

% 2. Parse Input (Integer Road ID or Continuous RSI)
if input_val >= 1 && input_val <= 4 && floor(input_val) == input_val
    % Discrete Road ID selection (1, 2, 3, 4)
    road_idx  = round(input_val);
    rsi_val   = rsi_nodes(road_idx);
    k1        = k1_nodes(road_idx);
    k2        = k2_nodes(road_idx);
    c1        = lambda_nodes(road_idx);
    mode_name = names_nodes{road_idx};
else
    % Continuous RSI from Edge AI Perception (e.g. RSI = 0.72)
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

end
