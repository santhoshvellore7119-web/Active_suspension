% SETUP_AS
% Master Parameter Configuration Script for Quanser Active Suspension System
% Dedicated Super-Twisting Sliding Mode Control (STSMC) & Adaptive Gain Scheduling
% Aligned with Professor Specifications & Reference Benchmark

fprintf('========================================================================\n');
fprintf('  QUANSER ACTIVE SUSPENSION SYSTEM: CONTROLLER SETUP\n');
fprintf('========================================================================\n');

%% 1. Plant Physical Parameters (Quanser Quarter-Car Benchmark)
ms   = 2.45;     % Sprung mass / Vehicle body mass (kg)
mu   = 1.00;     % Unsprung mass / Tire & wheel mass (kg)
mus  = mu;       % Alias for compatibility with original Quanser models
bs   = 7.5;      % Suspension inherent damping coefficient (N-s/m)
bus  = 5.0;      % Tire inherent damping coefficient (N-s/m)

if ~exist('SPRING_CONFIG', 'var')
    SPRING_CONFIG = 1;
end

if SPRING_CONFIG == 2
    ks = 490.0;
    kt = 1020.0;
else
    ks = 900.0;
    kt = 2500.0;
end
kus  = kt;       % Alias for tire stiffness

% Hardware Operational Limits
Fc_max = 39.2;   % Actuator maximum continuous force limit (N) [+/- 39.2 N]
Fc_min = -39.2;  % Actuator minimum force limit (N)
St_max = 0.038;  % Physical mechanical suspension travel limit (m) [38 mm]
St_min = -0.038; % Maximum negative suspension travel (m)
Se     = 0.000942; % Suspension optical encoder resolution (m/count)
Sm     = 0.112;    % Motor torque constant (N-m/A)
As     = 9.81;     % Accelerometer sensitivity (m/s^2/V)

%% 2. Continuous-Time State-Space Representation
A = [ 0,                   1,       0,                      -1;
     -ks/ms,             -bs/ms,    0,                     bs/ms;
      0,                   0,       0,                       1;
      ks/mu,              bs/mu,  -kt/mu,          -(bs + bus)/mu ];

B = [ 0,         0;
      0,       1/ms;
     -1,         0;
     bus/mu,  -1/mu ];

C = [ 1, 0, 0, 0;
      -ks/ms, -bs/ms, 0, bs/ms ];

D = zeros(6, 2);
D(6, 2) = 1/ms;

C_sim = cat(1, eye(4), C);
D_sim = D;

%% 3. Road Surface Excitation Profile Parameters
road_names = { ...
    'Smooth road', ...
    'Normal road', ...
    'Rough road', ...
    'Bumpy road', ...
    'Pulse road', ...
    'Potholes road' ...
};

if ~exist('ROAD_PROFILE_ID', 'var') || isempty(ROAD_PROFILE_ID)
    if exist('ROAD_ID', 'var') && ~isempty(ROAD_ID)
        ROAD_PROFILE_ID = ROAD_ID;
    else
        ROAD_PROFILE_ID = 1; % Default: Smooth road
    end
end
ROAD_ID = ROAD_PROFILE_ID;

%% 4. Super-Twisting Sliding Mode Controller (STSMC) Gains & Parameters
eps_bl = 0.01; % Uniform boundary layer for smooth tanh(s/eps)

if ~exist('STSMC_MODE', 'var') || isempty(STSMC_MODE)
    STSMC_MODE = 'adaptive';
end

if strcmp(STSMC_MODE, 'adaptive')
    [k1, k2, c1, rsi_val, mode_name] = stsmc_rsi_scheduler(ROAD_PROFILE_ID, 'scaled');
elseif strcmp(STSMC_MODE, 'conservative')
    c1 = 5.0;  k1 = 20.0;  k2 = 8.0;
elseif strcmp(STSMC_MODE, 'aggressive')
    c1 = 15.0; k1 = 180.0; k2 = 90.0;
else % moderate
    c1 = 7.0;  k1 = 60.0;  k2 = 25.0;
end

%% 5. Simulation Timing and Solver Settings
T_start  = 0.0;
T_stop   = 10.0;
RelTol   = 1e-4;
MaxStep  = 1e-3;

fprintf('Plant Parameters:\n');
fprintf('  Sprung Mass (ms): %.2f kg, Unsprung Mass (mu): %.2f kg\n', ms, mu);
fprintf('  Suspension Stiffness (ks): %.1f N/m, Tire Stiffness (kt): %.1f N/m\n', ks, kt);
fprintf('  Mechanical Travel Limit: +/- %.3f m (38 mm)\n', St_max);
fprintf('  Actuator Force Limit:   +/- %.1f N\n', Fc_max);
fprintf('Controller Configuration:\n');
fprintf('  Selected Road Profile [%d]: %s\n', ROAD_PROFILE_ID, road_names{ROAD_PROFILE_ID});
fprintf('  Mode: %s\n', STSMC_MODE);
fprintf('  Sliding Surface Gain (c1): %.2f\n', c1);
fprintf('  Proportional Gain   (k1): %.2f\n', k1);
fprintf('  Integral Gain       (k2): %.2f\n', k2);
fprintf('  Boundary Layer (eps_bl):   %.4f\n', eps_bl);
fprintf('========================================================================\n\n');

%% 6. Road Disturbance Timeseries Objects for Simulink FromWorkspace Blocks
t_sim = (0:0.001:T_stop)';

% 1. Smooth road: 4 mm, 0.20 Hz Sine Wave
z_smooth = 0.004 * sin(2 * pi * 0.20 * t_sim);
road_smooth = timeseries(z_smooth, t_sim);

% 2. Normal road: Harmonic near body resonance (2.58 Hz, 10 mm Sine)
z_normal = 0.010 * sin(2 * pi * 2.58 * t_sim);
road_normal = timeseries(z_normal, t_sim);

% 3. Rough road: 0.1 to 15.0 Hz Chirp Frequency Sweep
f0_chirp = 0.1; f1_chirp = 15.0; k_chirp = (f1_chirp - f0_chirp) / T_stop;
z_rough  = 0.010 * sin(2 * pi * (f0_chirp + 0.5 * k_chirp * t_sim) .* t_sim);
road_rough = timeseries(z_rough, t_sim);

% 4. Bumpy road: Triangular Ramp Obstacle (20 mm peak)
z_bumpy = zeros(size(t_sim));
r1_up   = (t_sim >= 1.5 & t_sim <= 2.0);
z_bumpy(r1_up) = 0.020 * (t_sim(r1_up) - 1.5) / 0.5;
r1_down = (t_sim > 2.0 & t_sim <= 2.5);
z_bumpy(r1_down) = 0.020 * (1.0 - (t_sim(r1_down) - 2.0) / 0.5);

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

% 6. Potholes road: Broadband ISO Class D + Localized Deep Potholes
rng(42);
n_harm = 30;
w_harm = 2*pi*linspace(0.5, 15.0, n_harm);
phi_harm = rand(1, n_harm) * 2 * pi;
amp_harm = 0.012 * (1 ./ (1:n_harm).^0.9);
amp_harm = amp_harm / sum(amp_harm) * 0.012;
z_potholes = zeros(size(t_sim));
for k = 1:n_harm
    z_potholes = z_potholes + amp_harm(k) * sin(w_harm(k)*t_sim + phi_harm(k));
end
z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  = z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  - 0.018; 
z_potholes(t_sim >= 5.0 & t_sim <= 5.15) = z_potholes(t_sim >= 5.0 & t_sim <= 5.15) - 0.020; 
z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  = z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  - 0.018; 
road_potholes = timeseries(z_potholes, t_sim);
