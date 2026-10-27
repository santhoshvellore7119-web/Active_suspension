function [K, P] = lqr_native(A, B, Q, R)
% LQR_NATIVE Solves the continuous-time Algebraic Riccati Equation (ARE)
% and computes the optimal state-feedback gain K in pure base MATLAB.
% Works without requiring the Control System Toolbox.
%
%   A' * P + P * A - P * B * inv(R) * B' * P + Q = 0
%   K = inv(R) * B' * P

    n = size(A, 1);
    % Form Hamiltonian matrix
    H = [ A,              -B * (R \ B');
         -Q,              -A' ];

    % Real Schur decomposition of Hamiltonian
    [U, T] = schur(H, 'real');
    [U, T] = ordschur(U, T, 'lhp'); % Order stable eigenvalues to left-half plane

    U11 = U(1:n, 1:n);
    U21 = U(n+1:2*n, 1:n);

    % Unique symmetric positive-definite Riccati solution P
    P = real(U21 / U11);
    P = (P + P') / 2; % Ensure exact numerical symmetry

    % Optimal state-feedback gain matrix
    K = R \ (B' * P);
end
