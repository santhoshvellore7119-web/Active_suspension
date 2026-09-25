function [u, w_int_next, s] = hil_stsmc_discrete(x_meas, w_int, c1, k1, k2, Ts, eps_bl)
%#codegen
% HIL_STSMC_DISCRETE Real-Time Deterministic STSMC Algorithm for Quanser Rig
%
% Designed for discrete-time Hardware-in-the-Loop (HIL) at Ts = 1 ms (1000 Hz).
%
% Inputs:
%   x_meas     : 4x1 measured state vector from hardware sensors / filters:
%                x1 = suspension travel (from optical encoder, m)
%                x2 = body velocity (from filtered derivative of encoder, m/s)
%                x3 = tire deflection (from bottom optical encoder, m)
%                x4 = wheel velocity (from filtered derivative, m/s)
%   w_int      : Scalar integrator state from previous sample step
%   c1         : Sliding surface parameter (lambda)
%   k1         : Convergence proportional gain
%   k2         : Robustness integral gain
%   Ts         : Discrete sample time (e.g. 0.001 s)
%   eps_bl     : Boundary layer smoothing factor (1e-3)
%
% Outputs:
%   u          : Saturated motor control force command (N) [-39.2, +39.2]
%   w_int_next : Integrator state for next sample step
%   s          : Sliding manifold value

if nargin < 7 || isempty(eps_bl)
    eps_bl = 1e-3;
end
if nargin < 6 || isempty(Ts)
    Ts = 0.001;
end

% 1. Extract states
x1 = x_meas(1);
x2 = x_meas(2);

% 2. Compute sliding surface
s = x2 + c1 * x1;

% 3. Boundary-layer smoothed switching function (Chattering suppression)
sign_s = s / (abs(s) + eps_bl);

% 4. Super-twisting control law
u_raw = -k1 * sqrt(abs(s)) * sign_s + w_int;

% 5. Actuator saturation (+/- 39.2 N)
Fc_limit = 39.2;
if u_raw > Fc_limit
    u = Fc_limit;
elseif u_raw < -Fc_limit
    u = -Fc_limit;
else
    u = u_raw;
end

% 6. Anti-windup discrete Euler integration of auxiliary state
w_int_dot = -k2 * sign_s;
if (u_raw >= Fc_limit && w_int_dot > 0) || (u_raw <= -Fc_limit && w_int_dot < 0)
    w_int_next = w_int; % Freeze integration (Anti-windup)
else
    w_int_next = w_int + w_int_dot * Ts;
end

% Bound auxiliary integrator state to avoid unbounded drift
w_int_limit = 39.2;
if w_int_next > w_int_limit
    w_int_next = w_int_limit;
elseif w_int_next < -w_int_limit
    w_int_next = -w_int_limit;
end

end
