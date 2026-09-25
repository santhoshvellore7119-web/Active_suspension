% DESIGN_STSMC
% Design and configuration script for Super-Twisting Sliding Mode Controller

function [c1_out, k1_out, k2_out] = design_stsmc(mode_choice)
    if nargin < 1 || isempty(mode_choice)
        mode_choice = 'moderate';
    end
    
    eps_bl = 1e-3;
    
    switch lower(mode_choice)
        case 'conservative'
            c1 = 0.0;  k1 = 20.0;   k2 = 10.0;
            desc = 'Conservative (Min travel, high safety)';
        case 'moderate'
            c1 = 0.0;  k1 = 100.0;  k2 = 50.0;
            desc = 'Moderate (Recommended starting baseline)';
        case 'aggressive'
            c1 = 0.0;  k1 = 200.0;  k2 = 100.0;
            desc = 'Aggressive (Max body acceleration suppression)';
        case 'grant_smooth'
            c1 = 5.0;  k1 = 8.0;    k2 = 2.0;
            desc = 'Grant Table - Smooth Road (RSI 0.10)';
        case 'grant_normal'
            c1 = 7.0;  k1 = 12.0;   k2 = 4.0;
            desc = 'Grant Table - Normal Road (RSI 0.35)';
        case 'grant_rough'
            c1 = 10.0; k1 = 18.0;   k2 = 7.0;
            desc = 'Grant Table - Rough Road (RSI 0.65)';
        case 'grant_pothole'
            c1 = 15.0; k1 = 25.0;   k2 = 10.0;
            desc = 'Grant Table - Pothole Road (RSI 0.95)';
        case 'rsi_adaptive'
            if evalin('base', 'exist(''ROAD_PROFILE_ID'', ''var'')')
                r_id = evalin('base', 'ROAD_PROFILE_ID');
            else
                r_id = 2;
            end
            [k1, k2, c1, rsi_val, mode_name] = stsmc_rsi_scheduler(r_id, 'scaled');
            desc = sprintf('RSI-Adaptive: %s (RSI=%.2f)', mode_name, rsi_val);
        otherwise
            error('Unknown STSMC mode: %s', mode_choice);
    end
    
    assignin('base', 'c1', c1);
    assignin('base', 'k1', k1);
    assignin('base', 'k2', k2);
    assignin('base', 'eps_bl', eps_bl);
    assignin('base', 'STSMC_MODE', mode_choice);
    
    fprintf('=== STSMC Controller Configured: %s ===\n', desc);
    fprintf('  Sliding Surface (c1 / lambda): %.2f\n', c1);
    fprintf('  Convergence Gain (k1):         %.2f\n', k1);
    fprintf('  Robustness Gain (k2):          %.2f\n', k2);
    fprintf('  Boundary Layer (eps_bl):       %.1e\n\n', eps_bl);
    
    if nargout > 0
        c1_out = c1;
        k1_out = k1;
        k2_out = k2;
    end
end
