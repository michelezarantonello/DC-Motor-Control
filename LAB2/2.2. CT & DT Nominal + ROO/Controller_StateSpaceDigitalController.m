%% CT (Nominal) Reduced-Order Observer Setup

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


% vector V of unobservable states
V = [0,1];

new_state = [C_c;V];

% Since x = [theta_l; omega_l] already, T = I (no need for CoB)
T = eye(2);

y = C_c;
v = V;

A_11 = A_c(1,1);
A_12 = A_c(1,2);
A_21 = A_c(2,1);
A_22 = A_c(2,2);

B_1 = B_c(1,1);
B_2 = B_c(2,1);

omega_n = abs(lambda1);
p_obs = -5 * omega_n;     % observer pole 5x faster, real axis

L_obs = place(A_22, A_12,p_obs);

A_o = A_22 - L_obs*A_12;
B_o = [B_2-L_obs*B_1, (A_22-L_obs*A_12)*L_obs + A_21-L_obs*A_11];
C_o = T*[0;1];
D_o = T*[0,1;0,L_obs];



