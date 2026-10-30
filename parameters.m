% PARAMETERS.M
% SINGLE SOURCE OF TRUTH for Quarter-Car Active Suspension Project
% Defines all plant physical parameters, hardware limits, solver settings,
% and controller parameters (LQR, Fixed STSMC, Adaptive STSMC).
%
% 6 Standard Road Profiles from Professor Specifications:
%   Profile 1: Smooth road   -> 4 mm, 0.20 Hz Sine Wave
%   Profile 2: Normal road   -> Harmonic excitation near body resonance (2.58 Hz, 10 mm Sine)
%   Profile 3: Rough road    -> 0.1 - 15.0 Hz Frequency Chirp Sweep
%   Profile 4: Bumpy road    -> Triangular Ramp Obstacle (20 mm peak)
%   Profile 5: Pulse road    -> 20 mm Step Pulse Drops (0.25 Hz)
%   Profile 6: Potholes road -> Broadband ISO Class D Roughness + Localized Deep Potholes

rng(42); % Fixed random seed for strict reproducibility

%% 1. Quanser 2-DOF Quarter-Car Physical Plant Parameters
if ~exist('ms', 'var') || isempty(ms), ms = 2.45; end     % Sprung mass (kg) [Vehicle chassis quarter]
if ~exist('mu', 'var') || isempty(mu), mu = 1.00; end     % Unsprung mass (kg) [Wheel / axle assembly]
if ~exist('ks', 'var') || isempty(ks), ks = 900.0; end    % Suspension spring stiffness (N/m)
if ~exist('kt', 'var') || isempty(kt), kt = 2500.0; end   % Tire vertical stiffness (N/m)
if ~exist('bs', 'var') || isempty(bs), bs = 7.5; end      % Suspension viscous damping (N-s/m)
if ~exist('bus', 'var') || isempty(bus), bus = 5.0; end   % Tire inherent damping (N-s/m)

%% 2. Hardware Physical Safety Limits
F_max      = 39.2;    % Actuator continuous force saturation limit (+/- N)
travel_max = 0.038;   % Mechanical rail stroke travel limit (+/- m, 38 mm)

%% 3. Simulation & Solver Configuration
T_stop   = 10.0;     % Total simulation duration (s)
dt       = 0.001;    % Discrete step size for analysis (s, 1000 Hz)
eps_smc  = 0.010;    % UNIFORM smoothing boundary layer for tanh(s/eps)
t_span   = (0:dt:T_stop)';
t_sim    = t_span;   % Alias for compatibility with reference repository scripts

%% 4. Road Disturbance Excitation Suite (Exact 6 Professor Profiles)
if ~exist('ROAD_ID', 'var') || isempty(ROAD_ID)
    ROAD_ID = 1; % Default: Smooth road
end

road_names = { ...
    'Smooth road', ...
    'Normal road', ...
    'Rough road', ...
    'Bumpy road', ...
    'Pulse road', ...
    'Potholes road' ...
};

% 1. Smooth road: 4 mm, 0.20 Hz Sine Wave
road_smooth_amp  = 0.004;    % Amplitude (m) [4 mm]
road_smooth_freq = 0.20;     % Frequency (Hz)
z_smooth = road_smooth_amp * sin(2 * pi * road_smooth_freq * t_sim);
road_smooth = timeseries(z_smooth, t_sim);

% 2. Normal road: Harmonic excitation near chassis body bounce resonance (2.58 Hz, 10 mm Sine)
road_normal_amp  = 0.010;    % Amplitude (m) [10 mm]
road_normal_freq = 2.58;     % Chassis bounce natural frequency (Hz)
z_normal = road_normal_amp * sin(2 * pi * road_normal_freq * t_sim);
road_normal = timeseries(z_normal, t_sim);

% 3. Rough road: 0.1 to 15.0 Hz Linear Frequency Chirp Sweep over 10 s
f0_chirp = 0.1;
f1_chirp = 15.0;
k_chirp  = (f1_chirp - f0_chirp) / T_stop;
z_rough  = 0.010 * sin(2 * pi * (f0_chirp + 0.5 * k_chirp * t_sim) .* t_sim);
road_rough = timeseries(z_rough, t_sim);

% 4. Bumpy road: Triangular Ramp Obstacle (20 mm Peak Obstacle)
z_bumpy = zeros(size(t_sim));
% Ramp 1 at t = 1.5s to 2.5s (0.5s ramp up to 20mm, 0.5s ramp down)
r1_up   = (t_sim >= 1.5 & t_sim <= 2.0);
z_bumpy(r1_up) = 0.020 * (t_sim(r1_up) - 1.5) / 0.5;
r1_down = (t_sim > 2.0 & t_sim <= 2.5);
z_bumpy(r1_down) = 0.020 * (1.0 - (t_sim(r1_down) - 2.0) / 0.5);

% Ramp 2 at t = 5.5s to 6.5s
r2_up   = (t_sim >= 5.5 & t_sim <= 6.0);
z_bumpy(r2_up) = 0.020 * (t_sim(r2_up) - 5.5) / 0.5;
r2_down = (t_sim > 6.0 & t_sim <= 6.5);
z_bumpy(r2_down) = 0.020 * (1.0 - (t_sim(r2_down) - 6.0) / 0.5);
road_bumpy = timeseries(z_bumpy, t_sim);

% 5. Pulse road: 20 mm Step Pulse Drops (0.25 Hz repetition, -20 mm drop)
z_pulse = zeros(size(t_sim));
for p_start = [1.0, 5.0]
    p_idx = (t_sim >= p_start & t_sim < p_start + 2.0);
    z_pulse(p_idx) = -0.020;
end
road_pulse = timeseries(z_pulse, t_sim);

% 6. Potholes road: Broadband ISO Class D Roughness + Localized Deep Potholes
rng(42); % Fixed seed for exact reproducibility
n_harm = 30;
w_harm = 2*pi*linspace(0.5, 15.0, n_harm);
phi_harm = rand(1, n_harm) * 2 * pi;
amp_harm = 0.012 * (1 ./ (1:n_harm).^0.9);
amp_harm = amp_harm / sum(amp_harm) * 0.012;
z_potholes = zeros(size(t_sim));
for k = 1:n_harm
    z_potholes = z_potholes + amp_harm(k) * sin(w_harm(k)*t_sim + phi_harm(k));
end
% Localized severe pothole drops
z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  = z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  - 0.018; 
z_potholes(t_sim >= 5.0 & t_sim <= 5.15) = z_potholes(t_sim >= 5.0 & t_sim <= 5.15) - 0.020; 
z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  = z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  - 0.018; 
road_potholes = timeseries(z_potholes, t_sim);

% Backward-compatibility variables for legacy code
road1_amp = road_smooth_amp; road1_freq = road_smooth_freq;
road2_amp = road_normal_amp; road2_freq = road_normal_freq;
road_rough_data = [t_span, z_rough];
road_rough_ts   = timeseries(z_rough, t_span);

%% 5. Continuous-Time State-Space Plant Matrices
plant_model;

%% 6. Controller 1: Linear Quadratic Regulator (LQR)
Q_lqr = diag([100.0, 100.0, 10.0, 10.0]); % State penalty matrix
R_lqr = 0.010;                             % Actuator force penalty
K_lqr = lqr_native(A, B(:, 2), Q_lqr, R_lqr);

%% 7. Controller 2: Fixed Super-Twisting SMC (Fixed STSMC)
c1_fixed = 7.00;    % Sliding surface slope: s = x2 + c1*x1
k1_fixed = 60.00;   % Proportional sliding gain
k2_fixed = 25.00;   % Integral sliding gain

%% 8. Controller 3: Adaptive Super-Twisting SMC (Adaptive STSMC)
if ~exist('OVERRIDE_ADAP_GAINS', 'var') || ~OVERRIDE_ADAP_GAINS
    [k1_adap, k2_adap, c1_adap, rsi_val, mode_name] = stsmc_rsi_scheduler(ROAD_ID, 'scaled');
end
