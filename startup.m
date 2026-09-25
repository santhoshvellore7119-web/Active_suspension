% STARTUP.M
% Automatic initialization when opening Active_Suspension_STSMC_Project in MATLAB / MATLAB Online

project_dir = fileparts(mfilename('fullpath'));
if isempty(project_dir)
    project_dir = pwd;
end
addpath(genpath(project_dir));
setup_as;
fprintf('Active Suspension STSMC Project initialized. Type "run_in_matlab_online" or "run_all_tests" to simulate.\n');
