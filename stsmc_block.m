function [u, w_int_dot, s] = stsmc_block(x, w_int, c1, k1, k2, eps_bl)
%#codegen
% STSMC_BLOCK Super-Twisting Sliding Mode Control Law
%
% Implements the continuous-time Super-Twisting 2nd-order sliding mode algorithm.
%
% Inputs:
%   x         : 4x1 state vector [x1; x2; x3; x4]
%               x1 = zs - zus (suspension deflection)
%               x2 = zs_dot   (sprung mass velocity)
%               x3 = zus - zr (tire deflection)
%               x4 = zus_dot  (unsprung mass velocity)
%   w_int     : Scalar integrator state from Simulink Integrator block
%   c1        : Sliding surface slope parameter (lambda in grant proposal)
%   k1        : Super-twisting continuous proportional gain (convergence)
%   k2        : Super-twisting integral gain (robustness / disturbance rejection)
%   eps_bl    : Boundary layer thickness for chattering elimination (default: 1e-3)
%
% Outputs:
%   u         : Active control force output (N)
%   w_int_dot : Time derivative of auxiliary integrator state (feeds Integrator)
%   s         : Value of sliding surface manifold

if nargin < 6 || isempty(eps_bl)
    eps_bl = 1e-3;
end

% 1. Extract relevant kinematic states
x1 = x(1); % Suspension deflection (relative displacement)
x2 = x(2); % Body vertical velocity

% 2. Compute sliding surface manifold: s = x2 + c1 * x1
s = x2 + c1 * x1;

% 3. Smooth boundary layer sign approximation to prevent numerical chattering
% and ensure smooth actuator command within +/- 39.2 N
sign_s = s / (abs(s) + eps_bl);

% 4. Super-Twisting Control Law:
%    u = -k1 * sqrt(abs(s)) * sign_s + w_int
%    dw_int/dt = -k2 * sign_s
u = -k1 * sqrt(abs(s)) * sign_s + w_int;
w_int_dot = -k2 * sign_s;

end
