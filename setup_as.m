% SETUP_AS
% Master Parameter Configuration Script for Quanser Active Suspension System
% Dedicated Super-Twisting Sliding Mode Control (STSMC) & Adaptive Gain Scheduling
% Aligned with ananthu0987/Roadprofile_CSelec and VIT-EMPOWER Grant 2026-27

fprintf('========================================================================\n');
fprintf('  QUANSER ACTIVE SUSPENSION SYSTEM: CONTROLLER SETUP\n');
fprintf('========================================================================\n');

%% 1. Plant Physical Parameters (Quanser Quarter-Car Benchmark)
ms   = 2.45;     % Sprung mass / Vehicle body mass (kg)
mu   = 1.00;     % Unsprung mass / Tire & wheel mass (kg)
mus  = mu;       % Alias for compatibility with original Quanser models
bs   = 7.5;      % Suspension inherent damping coefficient (N-s/m)
bus  = 5.0;      % Tire inherent damping coefficient (N-s/m)

% Spring stiffness configuration selector:
%   1 = Standard High-Stiffness (ks=900 N/m, kt=2500 N/m)
%   2 = Low-Stiffness Option    (ks=490 N/m, kt=1020 N/m)
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

% Quanser Hardware Specifications and Operational Limits
Fc_max = 39.2;   % Actuator maximum continuous force limit (N) [+/- 39.2 N]
Fc_min = -39.2;  % Actuator minimum force limit (N)
St_max = 0.038;  % Physical mechanical suspension travel limit (m) [38 mm]
St_min = -0.038; % Maximum negative suspension travel (m)
Se     = 0.000942; % Suspension optical encoder resolution (m/count)
Sm     = 0.112;    % Motor torque constant (N-m/A)
As     = 9.81;     % Accelerometer sensitivity (m/s^2/V)

%% 2. Continuous-Time State-Space Representation
% State vector x = [x1; x2; x3; x4]:
%   x1 = zs - zus   : Suspension deflection (m) [Travel]
%   x2 = zs_dot     : Sprung mass vertical velocity (m/s)
%   x3 = zus - zr   : Tire deflection (m)
%   x4 = zus_dot    : Unsprung mass vertical velocity (m/s)

A = [ 0,                   1,       0,                      -1;
     -ks/ms,             -bs/ms,    0,                     bs/ms;
      0,                   0,       0,                       1;
      ks/mu,              bs/mu,  -kt/mu,          -(bs + bus)/mu ];

B = [ 0,         0;
      0,       1/ms;
     -1,         0;
     bus/mu,  -1/mu ];

% Output matrices for Simulink Open-Loop State-Space block:
% Outputs: [x1; x2; x3; x4; suspension_deflection; zs_ddot]
C = [ 1, 0, 0, 0;
      -ks/ms, -bs/ms, 0, bs/ms ];

D = zeros(6, 2);
D(6, 2) = 1/ms;

C_sim = cat(1, eye(4), C);
D_sim = D;

%% 3. Road Surface Excitation Profile Parameters
% 6 Standard Road Profiles from ananthu0987/Roadprofile_CSelec:
%   1: Harmonics Generator (Chirp / Multi-frequency Resonance)
%   2: Discrete Pulse Generator (Periodic Bump / Step Drop)
%   3: Continuous Signal Generator (Sinusoidal Road Undulation)
%   4: Smooth Road (ISO Class A Elevation Variations)
%   5: Normal Road (ISO Class B/C Rolling Undulations)
%   6: Pothole Road (Random Roughness + Severe Localized Dips)

road_harm_amp      = 0.015;  % Harmonic amplitude (m)
road_pothole_amp   = 0.020;  % Pulse amplitude (m) [20 mm]
road_pothole_freq  = 0.25;   % Pulse repetition frequency (Hz)
road_pothole_width = 50.0;   % Pulse width (% of period)
road_sine_amp      = 0.020;  % Sine bump amplitude (m)
road_sine_freq     = 0.20;   % Sine bump frequency (Hz)

if ~exist('ROAD_PROFILE_ID', 'var') || isempty(ROAD_PROFILE_ID)
    if exist('ROAD_ID', 'var') && ~isempty(ROAD_ID)
        ROAD_PROFILE_ID = ROAD_ID;
    else
        ROAD_PROFILE_ID = 4; % Default: Smooth Road
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

road_names = { ...
    'Harmonics Generator (Chirp 1-8 Hz, Resonant)', ...
    'Pulse Generator (20 mm Discrete Bump)', ...
    'Signal Generator (20 mm Rectified Sine)', ...
    'Smooth Road (5 mm Sine, Class A)', ...
    'Normal Road (Dual Sine, Class B/C)', ...
    'Pothole Road (Random Texture + Dips)' ...
};

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

% Profile 4: Smooth Road (ISO Class A equivalent: minor elevation variations)
z_smooth = 0.005 * sin(2 * pi * 0.5 * t_sim);
road_smooth = timeseries(z_smooth, t_sim);

% Profile 5: Normal Road (ISO Class B/C equivalent: mixed rolling undulations)
z_normal = 0.015 * sin(2 * pi * 1.2 * t_sim) + 0.008 * sin(2 * pi * 2.8 * t_sim);
road_normal = timeseries(z_normal, t_sim);

% Profile 6: Random Pothole Filled Road Profile
rng(42); % Fixed seed for exact reproducibility
z_potholes = 0.005 * randn(size(t_sim));
z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  = z_potholes(t_sim >= 2.0 & t_sim <= 2.2)  - 0.012; 
z_potholes(t_sim >= 5.0 & t_sim <= 5.15) = z_potholes(t_sim >= 5.0 & t_sim <= 5.15) - 0.015; 
z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  = z_potholes(t_sim >= 7.5 & t_sim <= 7.7)  - 0.012; 
road_potholes = timeseries(z_potholes, t_sim);
