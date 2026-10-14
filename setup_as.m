% SETUP_AS
% Master Parameter Configuration Script for Quanser Active Suspension System
% Unified LQR (Classical, Optimal/ACO, HS) & Super-Twisting Sliding Mode Control (STSMC)
%
% Fully supports comparative analysis between LQR and STSMC across 4 RSI Road Profiles.

fprintf('========================================================================\n');
fprintf('  QUANSER ACTIVE SUSPENSION SYSTEM: UNIFIED LQR & STSMC SETUP\n');
fprintf('========================================================================\n');

%% 1. Plant Physical Parameters (Quanser Quarter-Car Benchmark)
ms   = 2.45;     % Sprung mass / Vehicle body mass (kg)
mu   = 1.00;     % Unsprung mass / Tire & wheel mass (kg)
mus  = mu;       % Alias for Quanser courseware compatibility
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

% Quanser Hardware Physical Specifications & Safety Limits
Fc_max = 39.2;     % Actuator maximum continuous force limit (+/- 39.2 N)
Fc_min = -39.2;    % Actuator minimum force limit (N)
St_max = 0.038;    % Physical mechanical suspension travel limit (+/- 38 mm)
St_min = -0.038;   % Maximum negative suspension travel (m)
Se     = 0.000942; % Suspension optical encoder resolution (m/count)
Sm     = 0.112;    % Motor torque constant (N-m/A)
As     = 9.81;     % Accelerometer sensitivity (m/s^2/V)

%% 2. Continuous-Time State-Space Representation
% State vector x = [x1; x2; x3; x4]:
%   x1 = zs - zus   : Suspension deflection (m) [Travel / Stroke]
%   x2 = zs_dot     : Sprung mass vertical velocity (m/s)
%   x3 = zus - zr   : Dynamic tire deflection (m)
%   x4 = zus_dot    : Unsprung mass vertical velocity (m/s)
%
% Input vector u_vec = [zr_dot; Fc]:
%   zr_dot : Road disturbance vertical velocity (m/s)
%   Fc     : Active actuator control force (N)

A = [ 0,                   1,       0,                      -1;
     -ks/ms,             -bs/ms,    0,                     bs/ms;
      0,                   0,       0,                       1;
      ks/mu,              bs/mu,  -kt/mu,          -(bs + bus)/mu ];

B = [ 0,         0;
      0,       1/ms;
     -1,         0;
     bus/mu,  -1/mu ];

% Output matrices for Simulink Open-Loop State-Space blocks:
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
    ROAD_PROFILE_ID = 2; % Default: Normal Road (RSI 0.35)
end

%% 4. Linear Quadratic Regulator (LQR) Controller Synthesis
% Classical Quanser Laboratory LQR Gain (Standard Courseware Baseline):
K_classical = [6.5, 165.0, -2254.5, -52.0];

% Optimal / ACO-Tuned LQR Synthesis (from setup_as.m (1).txt):
%   Q balances: [suspension_deflection, body_velocity, tire_deflection, wheel_velocity]
Q_lqr = diag([81.4724, 95.2896, 18.8891, 45.6697]);
R_lqr = 0.0260;
try
    K_aco = lqr(A, B(:,2), Q_lqr, R_lqr);
catch
    % Fallback if Control System Toolbox is absent
    K_aco = [5.9876, 61.5162, -572.6319, -22.7318];
end
K_lqr_opt = K_aco;

% Harmony Search (HS) Tuned LQR Gain (from Dinesh Model Master_Dinesh_CV_results.slx):
K_hs = [88.2645, 439.3617, 414.7621, 8.3968];

% Active LQR Gain Selection
if ~exist('LQR_TYPE', 'var') || isempty(LQR_TYPE)
    LQR_TYPE = 'CLASSICAL'; % 'CLASSICAL', 'ACO', 'HS'
end

switch upper(LQR_TYPE)
    case 'ACO'
        K_lqr_active = K_aco;
    case 'HS'
        K_lqr_active = K_hs;
    otherwise
        K_lqr_active = K_classical;
end

%% 5. Super-Twisting Sliding Mode Controller (STSMC) Gains & Parameters
% Boundary layer thickness for continuous, chattering-free sliding
eps_bl = 0.010; 

% Fixed STSMC Mode Gains:
%   Conservative: Comfort-oriented lower gains
c1_cons = 5.0;  k1_cons = 20.0;  k2_cons = 8.0;

%   Moderate: Balanced baseline
c1_mod  = 7.0;  k1_mod  = 60.0;  k2_mod  = 25.0;

%   Aggressive: High disturbance rejection
c1_aggr = 15.0; k1_aggr = 180.0; k2_aggr = 90.0;

% Fixed default settings for Simulink blocks:
c1_fixed = c1_mod;
k1_fixed = k1_mod;
k2_fixed = k2_mod;

% Adaptive STSMC: Scheduled based on Road Severity Index (RSI)
[k1_adap, k2_adap, c1_adap] = stsmc_rsi_scheduler(ROAD_PROFILE_ID, 'scaled');

% Mode selector for active STSMC workspace variables:
if ~exist('STSMC_MODE', 'var') || isempty(STSMC_MODE)
    STSMC_MODE = 'adaptive';
end

switch lower(STSMC_MODE)
    case 'conservative'
        c1 = c1_cons; k1 = k1_cons; k2 = k2_cons;
    case 'aggressive'
        c1 = c1_aggr; k1 = k1_aggr; k2 = k2_aggr;
    case 'moderate'
        c1 = c1_mod;  k1 = k1_mod;  k2 = k2_mod;
    otherwise % 'adaptive'
        c1 = c1_adap; k1 = k1_adap; k2 = k2_adap;
end

%% 6. Master Controller Selection (CONTROLLER_ID)
%   1 = Passive Baseline (Zero Actuator Force)
%   2 = Classical Quanser LQR
%   3 = Optimal / ACO-Tuned LQR
%   4 = Harmony Search (HS) LQR
%   5 = Fixed STSMC (Moderate)
%   6 = Adaptive STSMC (RSI-Scheduled)
if ~exist('CONTROLLER_ID', 'var') || isempty(CONTROLLER_ID)
    CONTROLLER_ID = 6; % Default: Adaptive STSMC
end

%% 7. Simulation Timing and Solver Settings
T_start  = 0.0;
T_stop   = 10.0;
RelTol   = 1e-4;
MaxStep  = 1e-3;

road_names = {'Smooth (4 mm Sine)', 'Normal (20 mm Sine)', 'Rough (15 mm Harmonic)', 'Pothole (20 mm Pulse)'};
ctrl_names = {'Passive Baseline', 'Classical LQR', 'ACO-Tuned LQR', 'HS-Tuned LQR', 'Fixed STSMC', 'Adaptive STSMC'};

fprintf('Plant Parameters:\n');
fprintf('  Sprung Mass (ms): %.2f kg, Unsprung Mass (mu): %.2f kg\n', ms, mu);
fprintf('  Suspension Stiffness (ks): %.1f N/m, Tire Stiffness (kt): %.1f N/m\n', ks, kt);
fprintf('  Mechanical Travel Limit: +/- %.3f m (38 mm)\n', St_max);
fprintf('  Actuator Force Limit:   +/- %.1f N\n', Fc_max);
fprintf('Controller Configuration:\n');
fprintf('  Active Controller [%d]: %s\n', CONTROLLER_ID, ctrl_names{CONTROLLER_ID});
fprintf('  LQR Gains Available:\n');
fprintf('    Classical: [%.2f, %.2f, %.2f, %.2f]\n', K_classical);
fprintf('    ACO-Opt:   [%.2f, %.2f, %.2f, %.2f]\n', K_aco);
fprintf('    HS-Opt:    [%.2f, %.2f, %.2f, %.2f]\n', K_hs);
fprintf('  STSMC Configuration (Mode: %s):\n', STSMC_MODE);
fprintf('    Sliding Surface Gain (c1): %.2f\n', c1);
fprintf('    Proportional Gain   (k1): %.2f\n', k1);
fprintf('    Integral Gain       (k2): %.2f\n', k2);
fprintf('    Boundary Layer (eps_bl):   %.4f\n', eps_bl);
fprintf('  Selected Road Profile [%d]: %s\n', ROAD_PROFILE_ID, road_names{ROAD_PROFILE_ID});
fprintf('========================================================================\n\n');
