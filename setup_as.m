% SETUP_AS
% Master Parameter Configuration Script for Quanser Active Suspension System
% Dedicated Super-Twisting Sliding Mode Control (STSMC) & Adaptive Gain Scheduling
%
% Strictly STSMC - All LQR artifacts removed.

fprintf('========================================================================\n');
fprintf('  QUANSER ACTIVE SUSPENSION SYSTEM: PURE STSMC CONTROLLER SETUP\n');
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
road_smooth_amp  = 0.004;    % Amplitude (m) [4 mm]
road_smooth_freq = 0.20;     % Frequency (Hz)

road_normal_amp  = 0.020;    % Amplitude (m) [20 mm]
road_normal_freq = 0.20;     % Frequency (Hz)

road_rough_amp   = 0.015;    % Peak amplitude (m) [15 mm]
road_rough_speed = 25.0;     % Vehicle forward speed (m/s) [90 km/h]

road_pothole_amp   = 0.020;  % Pulse amplitude (m) [20 mm]
road_pothole_freq  = 0.30;   % Pulse repetition frequency (Hz)
road_pothole_width = 10.0;   % Pulse width (% of period)

if ~exist('ROAD_PROFILE_ID', 'var') || isempty(ROAD_PROFILE_ID)
    ROAD_PROFILE_ID = 2; % Default: Normal Road
end

%% 4. Super-Twisting Sliding Mode Controller (STSMC) Gains & Parameters
% Boundary layer thickness for continuous chattering-free sliding
eps_bl = 0.01; 

% Mode selector:
%   'adaptive'     = Dynamic gain scheduling based on Road Severity Index (RSI)
%   'conservative' = Comfort-oriented lower gains (c1=5.0, k1=20.0, k2=8.0)
%   'moderate'     = Balanced performance (c1=7.0, k1=60.0, k2=25.0)
%   'aggressive'   = High disturbance rejection (c1=15.0, k1=180.0, k2=90.0)
if ~exist('STSMC_MODE', 'var') || isempty(STSMC_MODE)
    STSMC_MODE = 'adaptive';
end

if strcmp(STSMC_MODE, 'adaptive')
    % Automatically schedule STSMC gains based on Road Profile / RSI
    [k1, k2, c1] = stsmc_rsi_scheduler(ROAD_PROFILE_ID, 'scaled');
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

road_names = {'Smooth (4 mm Sine)', 'Normal (20 mm Sine)', 'Rough (15 mm Harmonic)', 'Pothole (20 mm Pulse)'};
fprintf('Plant Parameters:\n');
fprintf('  Sprung Mass (ms): %.2f kg, Unsprung Mass (mu): %.2f kg\n', ms, mu);
fprintf('  Suspension Stiffness (ks): %.1f N/m, Tire Stiffness (kt): %.1f N/m\n', ks, kt);
fprintf('  Mechanical Travel Limit: +/- %.3f m (38 mm)\n', St_max);
fprintf('  Actuator Force Limit:   +/- %.1f N\n', Fc_max);
fprintf('STSMC Active Configuration:\n');
fprintf('  Selected Road Profile [%d]: %s\n', ROAD_PROFILE_ID, road_names{ROAD_PROFILE_ID});
fprintf('  Mode: %s\n', STSMC_MODE);
fprintf('  Sliding Surface Gain (c1): %.2f\n', c1);
fprintf('  Proportional Gain   (k1): %.2f\n', k1);
fprintf('  Integral Gain       (k2): %.2f\n', k2);
fprintf('  Boundary Layer (eps_bl):   %.4f\n', eps_bl);
fprintf('========================================================================\n\n');

%% --- Road Profile Extensions ---
% Simulation time vector matching system sample time
t_sim = 0:0.001:10; % adjust end time to your sim stop time

% 1. Smooth Road Profile (Class A ISO equivalent: minor long-wavelength elevation variations)
% Amplitude ~ 5 mm (0.005 m) at gentle frequency
z_smooth = 0.005 * sin(2 * pi * 0.5 * t_sim);
road_smooth = timeseries(z_smooth(:), t_sim(:));

% 2. Normal Road Profile (Class B/C ISO equivalent: mixed gentle rolling undulations)
% Amplitudes ~ 15-20 mm
z_normal = 0.015 * sin(2 * pi * 1.2 * t_sim) + 0.008 * sin(2 * pi * 2.8 * t_sim);
road_normal = timeseries(z_normal(:), t_sim(:));

% 3. Random Pothole Filled Road Profile
% Baseline road roughness + localized negative dips (potholes)
rng(42); % repeatable seed
z_potholes = 0.005 * randn(size(t_sim)); % subtle surface texture
% Pothole pulses (negative displacement, e.g., -40 mm to -60 mm depth)
% Scaled Pothole pulses for benchtop 39.2N motor
z_potholes(t_sim >= 2.0 & t_sim <= 2.2) = z_potholes(t_sim >= 2.0 & t_sim <= 2.2) - 0.012; 
z_potholes(t_sim >= 5.0 & t_sim <= 5.15) = z_potholes(t_sim >= 5.0 & t_sim <= 5.15) - 0.015; 
z_potholes(t_sim >= 7.5 & t_sim <= 7.7) = z_potholes(t_sim >= 7.5 & t_sim <= 7.7) - 0.012; 
road_potholes = timeseries(z_potholes(:), t_sim(:));
