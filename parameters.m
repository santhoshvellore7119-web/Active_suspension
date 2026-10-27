% PARAMETERS.M
% SINGLE SOURCE OF TRUTH for Quarter-Car Active Suspension Project
% Defines all plant physical parameters, hardware limits, solver settings,
% and controller parameters (LQR, Fixed STSMC, Adaptive STSMC).
%
% Road Profiles from Reference Repository (Active_Suspension_STSMC_Project):
%   Profile 1: Smooth Road (RSI 0.10)   -> 4 mm Sine, 0.20 Hz
%   Profile 2: Normal Road (RSI 0.35)   -> 20 mm Sine, 0.20 Hz
%   Profile 3: Rough Road (RSI 0.65)    -> 15 mm ISO Class D Broadband (30 harmonics, rng 42)
%   Profile 4: Pothole Road (RSI 0.95)  -> 20 mm Pulse Drops (0.30 Hz, 10% width)
%   Profile 5: Harmonic Resonance Road  -> 10 mm Sine at 2.58 Hz (Quanser Body Hop Mode)

rng(42); % Fixed random seed for strict reproducibility

%% 1. Quanser 2-DOF Quarter-Car Physical Plant Parameters
if ~exist('ms', 'var') || isempty(ms), ms = 2.45; end     % Sprung mass (kg) [Vehicle chassis quarter]
if ~exist('mu', 'var') || isempty(mu), mu = 1.00; end     % Unsprung mass (kg) [Wheel / axle assembly]
if ~exist('ks', 'var') || isempty(ks), ks = 900.0; end    % Suspension spring stiffness (N/m)
if ~exist('kt', 'var') || isempty(kt), kt = 2500.0; end   % Tire vertical stiffness (N/m)
if ~exist('bs', 'var') || isempty(bs), bs = 7.5; end      % Suspension viscous damping (N-s/m)
if ~exist('bus', 'var') || isempty(bus), bus = 5.0; end   % Tire inherent damping (N-s/m)

%% 2. Hardware Physical Safety Limits
F_max     = 39.2;    % Actuator continuous force saturation limit (+/- N)
travel_max = 0.038;   % Mechanical rail stroke travel limit (+/- m, 38 mm)

%% 3. Simulation & Solver Configuration
T_stop   = 10.0;     % Total simulation duration (s)
dt       = 0.001;    % Discrete step size for analysis (s, 1000 Hz)
eps_smc  = 0.010;    % UNIFORM smoothing boundary layer for tanh(s/eps)
t_span   = (0:dt:T_stop)';

%% 4. Road Disturbance Excitation Suite (Exact Repository Profiles)
% Default active profile: 1 (Smooth), 2 (Normal), 3 (Rough ISO Class D), 4 (Potholes), 5 (Resonance)
if ~exist('ROAD_ID', 'var') || isempty(ROAD_ID)
    ROAD_ID = 1;
end

% Profile 1: Smooth Road (RSI 0.10) - Repository exact
road_smooth_amp  = 0.004;    % Amplitude (m) [4 mm]
road_smooth_freq = 0.20;     % Frequency (Hz)
road1_amp  = road_smooth_amp;
road1_freq = road_smooth_freq;

% Profile 2: Normal / Bumpy Road (RSI 0.35) - Repository exact
road_normal_amp  = 0.020;    % Amplitude (m) [20 mm]
road_normal_freq = 0.20;     % Frequency (Hz)
road2_amp  = road_normal_amp;
road2_freq = road_normal_freq;

% Profile 3: Rough Road / ISO Class D Broadband (RSI 0.65) - Repository exact
road_rough_amp   = 0.015;    % Peak amplitude (m) [15 mm]
road_rough_speed = 25.0;     % Vehicle forward speed (m/s) [90 km/h]
road3_amp  = road_rough_amp;

% Multi-harmonic ISO Class D synthesis matching repository run_comparative_sweep.m
rng(42); % Fixed seed for exact reproducibility matching repository
n_harmonics = 30;
omega_harm  = 2*pi*linspace(0.5, 15.0, n_harmonics);
phi_harm    = rand(1, n_harmonics) * 2 * pi;
amp_harm    = road_rough_amp * (1 ./ (1:n_harmonics).^0.9);
amp_harm    = amp_harm / sum(amp_harm) * road_rough_amp;

zr_rough = zeros(length(t_span), 1);
for k = 1:n_harmonics
    zr_rough = zr_rough + amp_harm(k) * sin(omega_harm(k)*t_span + phi_harm(k));
end
road_rough_data = [t_span, zr_rough];
road_rough_ts   = timeseries(zr_rough, t_span);

% Profile 4: Pothole / Pulse Road (RSI 0.95) - Repository exact
road_pothole_amp   = 0.020;  % Pulse amplitude (m) [20 mm]
road_pothole_freq  = 0.30;   % Pulse repetition frequency (Hz)
road_pothole_width = 10.0;   % Pulse width (% of period)
road4_amp   = road_pothole_amp;
road4_freq  = road_pothole_freq;
road4_width = road_pothole_width;

% Profile 5: Harmonic Body Hop Resonance (2.58 Hz)
road5_amp  = 0.010;          % 10 mm amplitude
road5_freq = 2.58;           % 2.58 Hz body bounce mode

%% 5. Continuous-Time State-Space Plant Matrices
% States: x1 = zs - zus (m), x2 = zs_dot (m/s), x3 = zus - zr (m), x4 = zus_dot (m/s)
% Inputs: u_vec = [zr_dot (m/s); u_force (N)]
plant_model;

%% 6. Controller 1: Linear Quadratic Regulator (LQR)
% Penalties: x1 (stroke), x2 (chassis velocity), x3 (tire deflection), x4 (wheel velocity)
Q_lqr = diag([100.0, 100.0, 10.0, 10.0]); % State penalty matrix
R_lqr = 0.010;                             % Actuator force penalty (prevent saturation)
K_lqr = lqr_native(A, B(:, 2), Q_lqr, R_lqr);

%% 7. Controller 2: Fixed Super-Twisting SMC (STSMC)
c1_fixed = 7.00;    % Sliding surface slope: s = x2 + c1*x1
k1_fixed = 60.00;   % Proportional sliding gain (super-twisting root branch)
k2_fixed = 25.00;   % Integral sliding gain (super-twisting integral branch)

%% 8. Controller 3: Adaptive Super-Twisting SMC (Adaptive STSMC)
% Scheduled dynamically via exact repository module: stsmc_rsi_scheduler.m
if ROAD_ID <= 4
    [k1_adap, k2_adap, c1_adap, rsi_val, mode_name] = stsmc_rsi_scheduler(ROAD_ID, 'scaled');
else
    [k1_adap, k2_adap, c1_adap, rsi_val, mode_name] = stsmc_rsi_scheduler(2, 'scaled');
end
