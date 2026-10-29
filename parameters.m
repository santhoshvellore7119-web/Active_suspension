% PARAMETERS.M
% SINGLE SOURCE OF TRUTH for Quarter-Car Active Suspension Project
% Defines all plant physical parameters, hardware limits, solver settings,
% and controller parameters (LQR, Fixed STSMC, Adaptive STSMC).
%
% 6 Standard Road Profiles from Reference Repository (ananthu0987/Roadprofile_CSelec):
%   Profile 1: Harmonics Generator (Chirp / Multi-frequency Resonance, 1-8 Hz sweep)
%   Profile 2: Discrete Pulse Generator (Periodic Bump/Step Drop, 20 mm, 0.25 Hz)
%   Profile 3: Signal Generator (Sinusoidal Road Bump, 20 mm Sine at 0.20 Hz, Rectified)
%   Profile 4: Smooth Road (ISO Class A Elevation Variations, 5 mm Sine at 0.50 Hz)
%   Profile 5: Normal Road (ISO Class B/C Rolling Undulations, Dual Sine 1.2 Hz & 2.8 Hz)
%   Profile 6: Pothole Road (Random Surface Roughness + Severe Localized Negative Dips)

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

%% 4. Road Disturbance Excitation Suite (6 Profiles from ananthu0987/Roadprofile_CSelec)
% Active profile selector: 1 to 6
if ~exist('ROAD_ID', 'var') || isempty(ROAD_ID)
    ROAD_ID = 4; % Default: Smooth Road
end

% Profile 1: Harmonics Generator Parameters (Chirp 1 to 8 Hz over 30 s)
road1_amp  = 0.015;  % Harmonic excitation peak amplitude (m)
road_harm_amp = road1_amp;

% Profile 2: Discrete Pulse Generator (20 mm Bump Drop)
road2_amp    = 0.020; % Pulse amplitude (m) [20 mm]
road2_period = 4.0;   % Period (s) [0.25 Hz]
road2_width  = 50.0;  % Pulse width (% of period)
road2_delay  = 0.1;   % Phase delay (s)
road_pulse_amp    = road2_amp;
road_pulse_period = road2_period;
road_pulse_width  = road2_width;
road_pulse_delay  = road2_delay;

% Profile 3: Signal Generator (Sinusoidal Bump with Half-Wave Saturation)
road3_amp  = 0.020;  % Amplitude (m) [20 mm]
road3_freq = 0.20;   % Frequency (Hz)
road_sine_amp  = road3_amp;
road_sine_freq = road3_freq;

% Profile 4: Smooth Road Profile (Class A ISO equivalent: minor long-wavelength elevation variations)
z_smooth = 0.005 * sin(2 * pi * 0.5 * t_sim);
road_smooth = timeseries(z_smooth, t_sim);

% Profile 5: Normal Road Profile (Class B/C ISO equivalent: mixed gentle rolling undulations)
z_normal = 0.015 * sin(2 * pi * 1.2 * t_sim) + 0.008 * sin(2 * pi * 2.8 * t_sim);
road_normal = timeseries(z_normal, t_sim);

% Profile 6: Random Pothole Filled Road Profile (Texture + Discrete Localized Dips)
rng(42); % Fixed seed for exact reproducibility
z_potholes = 0.005 * randn(size(t_sim)); % subtle surface texture
% Scaled pothole dips for benchtop actuator
z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  = z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  - 0.012; 
z_potholes(t_sim >= 5.0 & t_sim <= 5.15) = z_potholes(t_sim >= 5.0 & t_sim <= 5.15) - 0.015; 
z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  = z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  - 0.012; 
road_potholes = timeseries(z_potholes, t_sim);

% Backward-compatibility variables for legacy rough broadband definitions
road_rough_amp   = 0.015;
n_harmonics      = 30;
omega_harm       = 2*pi*linspace(0.5, 15.0, n_harmonics);
phi_harm         = rand(1, n_harmonics) * 2 * pi;
amp_harm         = road_rough_amp * (1 ./ (1:n_harmonics).^0.9);
amp_harm         = amp_harm / sum(amp_harm) * road_rough_amp;
zr_rough         = zeros(length(t_span), 1);
for k = 1:n_harmonics
    zr_rough = zr_rough + amp_harm(k) * sin(omega_harm(k)*t_span + phi_harm(k));
end
road_rough_data = [t_span, zr_rough];
road_rough_ts   = timeseries(zr_rough, t_span);

%% 5. Continuous-Time State-Space Plant Matrices
% States: x1 = zs - zus (m), x2 = zs_dot (m/s), x3 = zus - zr (m), x4 = zus_dot (m/s)
% Inputs: u_vec = [zr_dot (m/s); u_force (N)]
plant_model;

%% 6. Controller 1: Linear Quadratic Regulator (LQR)
% Penalties: x1 (stroke), x2 (chassis velocity), x3 (tire deflection), x4 (wheel velocity)
Q_lqr = diag([100.0, 100.0, 10.0, 10.0]); % State penalty matrix
R_lqr = 0.010;                             % Actuator force penalty (prevent saturation)
K_lqr = lqr_native(A, B(:, 2), Q_lqr, R_lqr);

%% 7. Controller 2: Fixed Super-Twisting SMC (Fixed STSMC)
c1_fixed = 7.00;    % Sliding surface slope: s = x2 + c1*x1
k1_fixed = 60.00;   % Proportional sliding gain (super-twisting root branch)
k2_fixed = 25.00;   % Integral sliding gain (super-twisting integral branch)

%% 8. Controller 3: Adaptive Super-Twisting SMC (Adaptive STSMC)
% Scheduled dynamically via exact repository module: stsmc_rsi_scheduler.m
[k1_adap, k2_adap, c1_adap, rsi_val, mode_name] = stsmc_rsi_scheduler(ROAD_ID, 'scaled');
