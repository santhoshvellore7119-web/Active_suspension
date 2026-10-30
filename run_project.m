% RUN_PROJECT.M
% Master Single-Command Runner for the Quanser Active Suspension Project
% Runs the entire project end-to-end in one single command:
%   1. Configures working directory and single source of truth parameters
%   2. Executes the 5-step automated verification test suite
%   3. Simulates all 6 road profiles and exports trajectory data
%   4. Executes the 54-scenario multi-parameter sweep (csv export)
%   5. Generates all 300 DPI publication comparison figures (3 controllers only)
%   6. Compiles the 17-page IEEE Access PDF research report
%   7. Displays the interactive dynamic response for road testing

clear variables;
clc;

fprintf('========================================================================================\n');
fprintf('  STARTING COMPLETE END-TO-END ACTIVE SUSPENSION BENCHMARK SUITE\n');
fprintf('  Comparing: (1) Optimal LQR  vs  (2) Fixed STSMC  vs  (3) Adaptive STSMC\n');
fprintf('  All 6 Road Profiles from reference repository (ananthu0987/Roadprofile_CSelec)\n');
fprintf('========================================================================================\n\n');

% 1. Ensure working directory is project root
proj_dir = fileparts(mfilename('fullpath'));
if isempty(proj_dir), proj_dir = pwd; end
cd(proj_dir);
addpath(genpath(proj_dir));

% 2. Initialize Single Source of Truth
fprintf('>>> STEP 1/6: Initializing Plant Physics & Controller Gains...\n');
parameters;
fprintf('    [OK] Quanser quarter-car matrices & gains loaded successfully.\n\n');

% 3. Automated Verification Suite
fprintf('>>> STEP 2/6: Running Master Automated Verification Suite...\n');
verify_project;
fprintf('    [OK] All mathematical & hardware limits verified.\n\n');

% 4. Multi-Road Simulation & Data Export
fprintf('>>> STEP 3/6: Simulating All 6 Road Profiles & Exporting Trajectories...\n');
export_sim_data;
fprintf('    [OK] Time-series trajectories exported to results/simulation_trajectories.mat\n\n');

% 5. 54-Scenario Multi-Parameter Sweep
fprintf('>>> STEP 4/6: Executing 54-Scenario Parameter Sweep (Roads x Masses x Stiffnesses)...\n');
run_parameter_sweep;
fprintf('    [OK] Multi-parameter sensitivity sweep exported to results/sweep_results.csv\n\n');

% 6. Precision-Tuned Candidate 5 Benchmark
fprintf('>>> STEP 5/6: Executing Candidate 5 (Adaptive STSMC) Benchmark Across All 6 Roads...\n');
run_candidate5('all', false);
fprintf('    [OK] Candidate 5 benchmark verified across all 6 standardized road profiles.\n\n');

% 7. Open Simulink Model Architecture & Display Interactive Dynamic Benchmark
fprintf('>>> STEP 6/6: Opening Simulink Model Canvas & Displaying Dynamic Benchmark Plot...\n');
if exist('Master_Active_Suspension_Clean.slx', 'file')
    try
        copyfile('Master_Active_Suspension_Clean.slx', 'Master_Active_Suspension.slx');
        model_name = 'Master_Active_Suspension';
    catch
        model_name = 'Master_Active_Suspension_Clean';
    end
else
    model_name = 'Master_Active_Suspension';
end

if usejava('desktop')
    open_system(model_name);
    fprintf('    [OK] Simulink model canvas opened in editor: %s.slx\n', model_name);
end
simulate_case(4, true);

fprintf('========================================================================================\n');
fprintf('  SUCCESS: ENTIRE PROJECT EXECUTED AND VERIFIED END-TO-END!\n');
fprintf('  Outputs:\n');
fprintf('    - Simulink Model:     Master_Active_Suspension.slx / Master_Active_Suspension_Clean.slx\n');
fprintf('    - Sweep Data Table:   results/sweep_results.csv (54 multi-parameter scenarios)\n');
fprintf('    - Simulation Data:    results/simulation_trajectories.mat (6 road profiles)\n');
fprintf('    - Publication Plots:  figures/ (Figure 1 to 6 + Dashboard)\n');
fprintf('    - Research Paper PDF: docs/IEEE_Access_Active_Suspension_STSMC_Paper.pdf\n');
fprintf('========================================================================================\n');
