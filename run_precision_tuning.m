function results = run_precision_tuning(road_id)
% RUN_PRECISION_TUNING Runs the Precision-Tuned Adaptive STSMC (Candidate 5)
% against Optimal LQR and Fixed STSMC across all 6 road profiles (or a chosen road)
% with NO PLOTS or figure windows (pure console metrics output).
%
% Usage:
%   run_precision_tuning          % Runs precision tuning for all 6 road profiles (no plots)
%   run_precision_tuning('all')   % Runs precision tuning for all 6 road profiles (no plots)
%   run_precision_tuning(1)       % Profile 1: Smooth road (no plots)
%   run_precision_tuning(2)       % Profile 2: Normal road (no plots)
%   run_precision_tuning(3)       % Profile 3: Rough road (no plots)
%   run_precision_tuning(4)       % Profile 4: Bumpy road (no plots)
%   run_precision_tuning(5)       % Profile 5: Pulse road (no plots)
%   run_precision_tuning(6)       % Profile 6: Potholes road (no plots)

    if nargin < 1 || isempty(road_id)
        results = run_candidate5('all', false);
    else
        results = run_candidate5(road_id, false);
    end
end
