% RUN_ALL_ROAD_PLOTS Opens Simulink model and opens plots for all 6 road profiles
% sequentially without closing the previous one, leaving all 6 figure windows open simultaneously.
%
% Usage:
%   run_all_road_plots

% Auto-align MATLAB directory to project folder
proj_dir = fileparts(mfilename('fullpath'));
if ~isempty(proj_dir) && ~strcmp(pwd, proj_dir)
    cd(proj_dir);
    addpath(genpath(proj_dir));
end

run_simulink_and_plots('all');
