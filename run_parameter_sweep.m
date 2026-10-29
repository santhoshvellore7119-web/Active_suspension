% RUN_PARAMETER_SWEEP.M
% Comprehensive multi-parameter sweep for the 3-controller Quarter-Car project
% Compares: (1) LQR, (2) Fixed Super-Twisting SMC, (3) Adaptive Super-Twisting SMC
% Sweeps:
%   - Road Profiles (5 profiles)
%   - Sprung Mass variations (Nominal, Loaded, Light)
%   - Suspension Stiffness variations (Nominal, Soft, Stiff)
% Exports full dataset to results/sweep_results.csv

clear variables;
clc;

fprintf('========================================================================================\n');
fprintf('  STARTING QUANSER ACTIVE SUSPENSION: COMPREHENSIVE PARAMETER SWEEP\n');
fprintf('  Evaluating LQR vs Fixed STSMC vs Adaptive STSMC across Multi-Parameter Matrix\n');
fprintf('========================================================================================\n\n');

if ~exist('results', 'dir'), mkdir('results'); end

% Initialize single source of truth
parameters;

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
if ~bdIsLoaded(model_name), load_system(model_name); end

% Define parameter variations
roads = [1, 2, 3, 4, 5, 6];
road_labels = {'Harmonics_Chirp', 'Pulse_Bump', 'SignalGen_Sine', 'Smooth_ClassA', 'Normal_ClassBC', 'Pothole_Dips'};

mass_variations = [2.45, 3.20, 1.95];
mass_labels     = {'Nominal_2.45kg', 'Loaded_3.20kg', 'Light_1.95kg'};

stiff_variations = [900.0, 700.0, 1100.0];
stiff_labels     = {'Nominal_900Nm', 'Soft_700Nm', 'Stiff_1100Nm'};

% Preallocate results table storage
rows = {};

total_runs = length(roads) * length(mass_variations) * length(stiff_variations);
run_idx = 0;

calc_rms = @(x) sqrt(mean(x.^2));
calc_pk  = @(x) max(abs(x));

fprintf('Executing %d sweep runs. Please wait...\n\n', total_runs);

for r = 1:length(roads)
    r_id = roads(r);
    r_name = road_labels{r};
    
    for m = 1:length(mass_variations)
        m_val = mass_variations(m);
        m_name = mass_labels{m};
        
        for k = 1:length(stiff_variations)
            k_val = stiff_variations(k);
            k_name = stiff_labels{k};
            
            run_idx = run_idx + 1;
            
            % Update workspace parameters
            ROAD_ID = r_id;
            ms = m_val;
            ks = k_val;
            assignin('base', 'ROAD_ID', r_id);
            assignin('base', 'ms', m_val);
            assignin('base', 'ks', k_val);
            
            % Recompute plant matrices and LQR gain for this plant variation
            parameters;
            
            % Run simulation
            simOut = sim(model_name);
            
            % Extract signals
            t = simOut.tout(:);
            idx_ss = (t >= t(end)*0.5);
            
            acc_pass = simOut.zs_ddot_pass(:);
            acc_lqr  = simOut.zs_ddot_lqr(:);
            acc_fsmc = simOut.zs_ddot_fsmc(:);
            acc_asmc = simOut.zs_ddot_asmc(:);
            
            def_pass = simOut.defl_pass(:) * 1000;  % mm
            def_lqr  = simOut.defl_lqr(:) * 1000;   % mm
            def_fsmc = simOut.defl_fsmc(:) * 1000;  % mm
            def_asmc = simOut.defl_asmc(:) * 1000;  % mm
            
            tir_pass = simOut.tire_pass(:) * 1000;  % mm
            tir_lqr  = simOut.tire_lqr(:) * 1000;   % mm
            tir_fsmc = simOut.tire_fsmc(:) * 1000;  % mm
            tir_asmc = simOut.tire_asmc(:) * 1000;  % mm
            
            u_pass = zeros(size(t));
            u_lqr  = simOut.u_lqr(:);
            u_fsmc = simOut.u_fsmc(:);
            u_asmc = simOut.u_asmc(:);

            % Guarantee signal dimensions match simulation time vector
            if length(acc_pass) ~= length(t), acc_pass = zeros(size(t)); end
            if length(acc_lqr)  ~= length(t), acc_lqr  = zeros(size(t)); end
            if length(acc_fsmc) ~= length(t), acc_fsmc = zeros(size(t)); end
            if length(acc_asmc) ~= length(t), acc_asmc = zeros(size(t)); end

            if length(def_pass) ~= length(t), def_pass = zeros(size(t)); end
            if length(def_lqr)  ~= length(t), def_lqr  = zeros(size(t)); end
            if length(def_fsmc) ~= length(t), def_fsmc = zeros(size(t)); end
            if length(def_asmc) ~= length(t), def_asmc = zeros(size(t)); end

            if length(tir_pass) ~= length(t), tir_pass = zeros(size(t)); end
            if length(tir_lqr)  ~= length(t), tir_lqr  = zeros(size(t)); end
            if length(tir_fsmc) ~= length(t), tir_fsmc = zeros(size(t)); end
            if length(tir_asmc) ~= length(t), tir_asmc = zeros(size(t)); end

            if length(u_pass) ~= length(t), u_pass = zeros(size(t)); end
            if length(u_lqr)  ~= length(t), u_lqr  = zeros(size(t)); end
            if length(u_fsmc) ~= length(t), u_fsmc = zeros(size(t)); end
            if length(u_asmc) ~= length(t), u_asmc = zeros(size(t)); end
            
            % Systems list: Passive + 3 Active Controllers
            ctrls = { ...
                'Passive',      acc_pass, def_pass, tir_pass, u_pass; ...
                'LQR',          acc_lqr,  def_lqr,  tir_lqr,  u_lqr;  ...
                'Fixed_STSMC',  acc_fsmc, def_fsmc, tir_fsmc, u_fsmc; ...
                'Adap_STSMC',   acc_asmc, def_asmc, tir_asmc, u_asmc  ...
            };
            
            for c = 1:4
                c_name = ctrls{c, 1};
                a_sig  = ctrls{c, 2};
                d_sig  = ctrls{c, 3};
                t_sig  = ctrls{c, 4};
                u_sig  = ctrls{c, 5};
                
                rms_a_ss = calc_rms(a_sig(idx_ss));
                rms_a    = calc_rms(a_sig);
                pk_a     = calc_pk(a_sig);
                rms_d_ss = calc_rms(d_sig(idx_ss));
                pk_d     = calc_pk(d_sig);
                rms_t_ss = calc_rms(t_sig(idx_ss));
                pk_t     = calc_pk(t_sig);
                pk_u     = calc_pk(u_sig);
                rms_u_ss = calc_rms(u_sig(idx_ss));
                
                rail_pass  = (pk_d <= 38.0);
                force_pass = (pk_u <= 39.201);
                
                rows{end+1} = { ...
                    r_name, m_val, k_val, c_name, ...
                    rms_a_ss, rms_a, pk_a, rms_d_ss, pk_d, rms_t_ss, pk_t, pk_u, rms_u_ss, ...
                    rail_pass, force_pass ...
                };
            end
            
            fprintf('[%2d/%2d] Road: %-15s | Mass: %.2f kg | Stiff: %4.0f N/m | Done\n', ...
                run_idx, total_runs, r_name, m_val, k_val);
        end
    end
end

% Build dataset table
headers = { ...
    'Road_Profile', 'Sprung_Mass_kg', 'Susp_Stiffness_Nm', 'Controller', ...
    'SS_RMS_Accel_ms2', 'Tot_RMS_Accel_ms2', 'Peak_Accel_ms2', ...
    'SS_RMS_Travel_mm', 'Peak_Travel_mm', 'SS_RMS_Tire_mm', 'Peak_Tire_mm', ...
    'Peak_Force_N', 'SS_RMS_Force_N', 'Rail_Limit_Passed', 'Motor_Limit_Passed' ...
};

csv_path = fullfile('results', 'sweep_results.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, '%s\n', strjoin(headers, ','));
for i = 1:length(rows)
    r = rows{i};
    fprintf(fid, '%s,%.2f,%.1f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%d,%d\n', ...
        r{1}, r{2}, r{3}, r{4}, r{5}, r{6}, r{7}, r{8}, r{9}, r{10}, r{11}, r{12}, r{13}, r{14}, r{15});
end
fclose(fid);

% Restore nominal plant parameters
clear ms ks ROAD_ID;
parameters;

fprintf('\n========================================================================================\n');
fprintf('  PARAMETER SWEEP COMPLETE! %d Runs Successfully Recorded\n', total_runs);
fprintf('  Dataset exported to: %s\n', csv_path);
fprintf('========================================================================================\n\n');
