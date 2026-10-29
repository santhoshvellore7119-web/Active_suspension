% PLANT_MODEL.M
% Builds continuous-time state-space matrices for the Quanser 2-DOF Quarter-Car
% States: x = [x1; x2; x3; x4]
%   x1 = zs - zus   : Suspension deflection (m)
%   x2 = zs_dot     : Sprung mass vertical velocity (m/s)
%   x3 = zus - zr   : Dynamic tire deflection (m)
%   x4 = zus_dot    : Unsprung mass vertical velocity (m/s)
% Inputs: u_in = [zr_dot; u_force]
%   u_in(1) = zr_dot  : Road disturbance vertical velocity (m/s)
%   u_in(2) = u_force : Active actuator force (N)

A = [ 0,                   1,       0,                      -1;
     -ks/ms,             -bs/ms,    0,                     bs/ms;
      0,                   0,       0,                       1;
      ks/mu,              bs/mu,  -kt/mu,          -(bs + bus)/mu ];

B = [ 0,         0;
      0,       1/ms;
     -1,         0;
     bus/mu,  -1/mu ];

% Output matrices for Simulink State-Space blocks:
% Outputs:
%   y(1:4) = [x1; x2; x3; x4]     : Full state vector
%   y(5)   = suspension deflection : zs - zus (m)
%   y(6)   = zs_ddot               : Sprung body acceleration (m/s^2)

C = [ 1, 0, 0, 0;
      -ks/ms, -bs/ms, 0, bs/ms ];

D = zeros(6, 2);
D(6, 2) = 1/ms;

C_sim = cat(1, eye(4), C);
D_sim = D;

% Controllability and Inherent Pole Analysis
B_ctrl = B(:, 2);
Co = [B_ctrl, A*B_ctrl, A^2*B_ctrl, A^3*B_ctrl];
ctrb_rank = rank(Co);
poles_open_loop = eig(A);

wn_body  = abs(poles_open_loop(3)); % rad/s
f_body   = wn_body / (2*pi);        % Hz
wn_wheel = abs(poles_open_loop(1)); % rad/s
f_wheel  = wn_wheel / (2*pi);       % Hz
