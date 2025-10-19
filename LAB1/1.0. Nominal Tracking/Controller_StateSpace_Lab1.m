%% SSM Control Design (Nominal)

% We calculate the eigenvalues lambda_1 and lambda_2,
% assuming a simplified system with dominant poles, 
% and they are functions of w_c and delta, which in turn we 
% calculate using t_s and M_p that we desire. To design
% the static feedback matrix K, we use 'place' or 'acker' routines
% to make sure the eigenvalues of the feedback system are
% exactly where lambda_1 and lambda_2 are pre-calculated.


% Motor SSM Realization:

A_c = [0  1 ; 0 -1/Tm];
B_c = [0 ; km/(Tm*N)];
C_c = [1 0];
D_c = 0;

% SSM of the Transfer Function of the Dominant-Pole Accurate Model:
StateSpace_simplified = ss(A_c, B_c, C_c, D_c);

M = [A_c B_c ; C_c D_c];

b = [0; 0; 1];

% Determining Nx, Nu by solving for M * [Nx Nu]^T = [0 1]^T

Sol = M\b;

Nx = Sol(1:2, :);
Nu = Sol(3, :);

% Required Performance Specifications
ts_ss = 0.15; 
mp_ss = 0.1; 
 
% Determining delta and omega_n from desired ts and mp:

delta = log(1/mp_ss) / sqrt(pi^2 + (log(1/mp_ss))^2);
omega_n = 3 / (delta * ts_ss);

% Use delta and omega_n to calculate the two eigenvalues:
lambda1 = -delta * omega_n + 1i * omega_n * sqrt(1 - delta^2);
lambda2 = -delta * omega_n - 1i * omega_n * sqrt(1 - delta^2);

p = [lambda1, lambda2];

%Routine for desired eigenvalue placement:
K_stateFB = place(A_c, B_c, p);

%The HPF (real derivative):
omega_c = 2*pi*50;
delta_H1 = 1/(sqrt(2));
H1.num = [omega_c^2,0];
H1.den = [1, 2*delta_H1 * omega_c, omega_c^2];
