%% B_eq and τ_sf estimation:

%% 1. Estimation for positive speed profile:
% Load data
load("Averaged_Positive_Torque_Profile.mat");          % variable: Torque
load("Averaged_Positive_Speed_Profile.mat");           % variable: Speed

gear_ratio = 1/14;

T_p = avg_pos_T;
S_p = avg_pos_S/gear_ratio;
N = length(T_p);

% Time vector (sampled at 1 ms)
t = (0:N-1)*0.001;  

Y_p = T_p';
%plot(t, T_p); % Visualize positive torque profile

% Regressor: include speed and its scaled sign
Speed_sign_p = gear_ratio*sign(S_p);
Phi_p = [S_p; Speed_sign_p]';

% Least squares estimate for positive-speed data
%ATTENTION: the averaged torque data is 43901 in length, while the speed is
%45001 long if not processed. For this reason I produce a new .mat file
%called "Averaged_Positive_Speed_Profile.mat" which is actually only
%spliced to remove the peaks and reducing its length in the meantime

thLS_p = Phi_p \ Y_p;    % [B_eq_p; τ_sf_p]      

%% 2. Estimation for negative speed profile:
load("Averaged_Negative_Torque_Profile.mat");         % variable: Torque
load("Averaged_Negative_Speed_Profile.mat");          % variable: Speed

T_n = avg_neg_T;
S_n = avg_neg_S/gear_ratio;

Y_n = T_n';  

Speed_sign_n = gear_ratio*sign(S_n);
Phi_n = [S_n; Speed_sign_n]';

% Least squares estimate for negative-speed data
thLS_n = Phi_n \ Y_n;    % [B_eq_n; τ_sf_n]

%% 3. Calculate the average estimates
% For B_eq we use the arithmetic average.
% For τ_sf, since the sign may be negative, take absolute values as needed.

B_eq_avg   = ( thLS_p(1) + thLS_n(1) ) / 2
tau_sf_avg = ( abs(thLS_p(2)) + abs(thLS_n(2)) ) / 2

thLS = [B_eq_avg; tau_sf_avg];

%% 4. Compute confidence intervals for the parameters
p = 2;  % number of parameters estimated (B_eq and tau_sf)

% --- Positive experiment ---
theta_p = thLS_p(:);             % ensure column vector [2×1]
res_p   = Y_p  - Phi_p * theta_p;  % residuals [N×1]
M_p     = numel(Y_p);
s2_p    = (res_p' * res_p) / (M_p - p);           % unbiased estimate of λ²
cov_p   = s2_p * inv(Phi_p' * Phi_p);             % 2×2 covariance matrix

% --- Negative experiment ---
theta_n = thLS_n(:);
res_n   = Y_n  - Phi_n * theta_n;
M_n     = numel(Y_n);
s2_n    = (res_n' * res_n) / (M_n - p);
cov_n   = s2_n * inv(Phi_n' * Phi_n);

% --- Combine the two estimates by averaging ---
B_eq_avg   = (theta_p(1) + theta_n(1)) / 2;
tau_sf_avg = (abs(theta_p(2)) + abs(theta_n(2))) / 2;

% Variance of the average of two independent estimates is 1/4*(var1 + var2)
var_B    = 0.25 * (cov_p(1,1) + cov_n(1,1));
var_tau  = 0.25 * (cov_p(2,2) + cov_n(2,2));

% --- Confidence intervals ---
alpha = 0.05;
% For large N you can use z=1.96; for smaller N use the t‑quantile:
tval = tinv(1 - alpha/2, min(M_p, M_n) - p);  

CI_B    = B_eq_avg   + [-1,1] * tval * sqrt(var_B);
CI_tau  = tau_sf_avg + [-1,1] * tval * sqrt(var_tau);

% --- Display results ---
fprintf('B_{eq} = %.5g  (95%% CI: [%.5g, %.5g])\n', B_eq_avg, CI_B(1), CI_B(2));
fprintf('\\tau_{sf} = %.5g  (95%% CI: [%.5g, %.5g])\n', tau_sf_avg, CI_tau(1), CI_tau(2));
