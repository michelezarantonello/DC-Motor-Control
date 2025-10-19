%% estimate_joint_params.m
% Estimates beam viscous friction B_b and joint stiffness k
% from the free‑oscillation ODE Jb*θdd + Bb*θd + k*θ = 0
% following Sec.5 handout (logarithmic decrement).


%% 1) Nominal/tentative parameters (from Table 1 & Listing 1)

Jb    = 1.4e-3;        % beam inertia [kg·m^2]
Bb    = 3.4e-3;        % tentative beam viscous friction [N·m/(rad/s)]
k     = 0.83;          % tentative joint stiffness [N·m/rad]
theta0 = 20 * pi/180;  % initial deflection [rad] (≤20° small‑angle)
omega0 = 0;            % initial angular rate

%% 2) Simulate the unforced ODE (hub locked, no disturbances)
%    Eq.(62): Jb*θdd + Bb*θd + k*θ = 0

odefun = @(t,y) [ y(2);
                 -(Bb/Jb)*y(2) - (k/Jb)*y(1) ];

tspan = [0, 1.5];        % 1.5 s should capture several oscillations
y0    = [theta0; omega0];
opts  = odeset('RelTol',1e-8,'AbsTol',1e-9);
[tt, yy] = ode45(odefun, tspan, y0, opts);

theta = yy(:,1);

%% 3) Extract M peaks of |θ(t)| to build data {k, log|θ(t_k)|}

M = 6;   % <- only dynamic: change when you have more lab peaks

% find local maxima of abs(theta)
[pk_vals, pk_locs] = findpeaks(abs(theta), tt, 'NPeaks',M, 'SortStr','descend');
% sort by time
[tk, ix] = sort(pk_locs);
pk_vals = pk_vals(ix);

% build regression data
k_idx = (0:M-1).';               % 0,1,2,...,M-1
Y      = log(pk_vals);           % log |θ(t_k)|

Phi    = [ -k_idx , ones(M,1) ]; % regressors for z = -a k + b

%% 4) LS fit for logarithmic decrement ξ

theta_ls = (Phi.'*Phi)\(Phi.'*Y); % = [a; b]
xi_hat   = theta_ls(1);           % slope = a = ξ
b_hat    = theta_ls(2);           % intercept (unused)

% damping ratio δ from Eq.(80)

delta = xi_hat / sqrt(pi^2 + xi_hat^2);

%% 5) Estimate damped ω and natural ω_n

Tks = diff(tk);                   % Tk = t_{k+1}-t_k
omega_d = pi ./ Tks;              % Eq.(81) for each interval
omega_d = mean(omega_d);          % average damped freq

omega_n = omega_d / sqrt(1 - delta^2);  % Eq.(83)

%% 6) Finally compute B_est_b and k_est from δ, ω_n, Jb

k_est   = Jb * omega_n^2;
Bb_est  = 2 * delta * omega_n * Jb;

%% 7.1) Plot θ(t) and |θ(t)| to visualize peaks with damping envelopes

% If the damping envelope bounds the signal well, our estimation is good 

sigma_hat = -delta * omega_n;     % decay rate

A_env = max(abs(theta));      % estimated amplitude envelope A

env_pos = A_env * exp(sigma_hat * tt);  % upper envelope
env_neg = -env_pos;                 % lower envelope

figure;
subplot(2,1,1)
plot(tt, theta*180/pi, 'b'); hold on
plot(tt, env_pos*180/pi, '--r', 'DisplayName','Envelope');
plot(tt, env_neg*180/pi, '--r', 'HandleVisibility','off');
ylabel('θ_d (deg)'), grid on
title('Free oscillation with damping envelope');
legend('θ_d(t)', 'Damping envelope');

subplot(2,1,2)
plot(tt, abs(theta)*180/pi, 'k');
ylabel('|θ_d| (deg)'), xlabel('t (s)'), grid on
title('Absolute value for peak detection');

%% 7.2) Compare envelope to true peak amplitudes (RMSE metric)

% Recompute estimated envelope at peak times
A_hat = pk_vals(1);  % estimated initial amplitude

% Align envelope to the first peak time ( the findpeak function otherwise
% misses the first true peak - at t0 = 0 for a cosine signal )
tk_rel = tk - tk(1);  % shift time axis so t0 = 0
env_est = A_hat * exp(sigma_hat * tk_rel);  % properly aligned


% RMSE between estimated envelope and true peaks
rmse_env = sqrt(mean((env_est - pk_vals).^2));
nRMSE_percent = (rmse_env / A_hat) * 100;

% Print result
fprintf('Envelope RMSE (vs true peaks): %.4e rad\n', rmse_env);

fprintf('Normalized envelope RMSE: %.2f %%\n', nRMSE_percent);

% Optional: plot comparison
figure;
plot(tk, pk_vals, 'ob', 'DisplayName','True peaks');
hold on;
plot(tk, env_est, 'xr', 'DisplayName','Estimated envelope');
grid on;
xlabel('t_k (s)');
ylabel('θ_d(t_k) [rad]');
title('True peaks vs. estimated envelope @ peak times');
legend('Location','northeast');

%% 8) Display results

fprintf('\n Estimated parameters (from %d peaks):\n', M);
fprintf('  damping ratio δ   = %.4f\n', delta);
fprintf('  damped freq ω     = %.2f rad/s\n', omega_d);
fprintf('  natural freq ω_n  = %.2f rad/s\n', omega_n);
fprintf('  stiffness k_est   = %.4f N·m/rad\n', k_est);
fprintf('  viscous B_b_est   = %.4e N·m/(rad/s)\n\n', Bb_est);

%% 9) (Optional) plot LS regression line

figure; hold on; grid on
plot(k_idx, Y, 'o', 'MarkerFaceColor','b');
plot(k_idx, Phi*theta_ls, '-r');
xlabel('k (peak index)');
ylabel('log|θ_d(t_k)|');
legend('data','LS fit');
title('Logarithmic decrement regression');

%% 10) Save results to CSV

% Define table with peak times and values
peak_table = table((0:M-1).', tk, pk_vals, log(pk_vals), ...
    'VariableNames', {'PeakIndex','Time_s','ThetaAbs_rad','LogThetaAbs'});

% Add estimated parameters as a second table row
est_table = table(delta, omega_d, omega_n, Bb_est, k_est, ...
    'VariableNames', {'delta','omega_d','omega_n','B_b_est','k_est'});

% Create filename with timestamp
timestamp = datestr(now,'yyyy_mm_dd_HH_MM_SS');
filename_peaks = ['peak_data_' timestamp '.csv'];
filename_est   = ['estimates_' timestamp '.csv'];

% Save to CSV
writetable(peak_table, filename_peaks);
writetable(est_table, filename_est);

fprintf('Saved peak data to    : %s\n', filename_peaks);
fprintf('Saved estimates to    : %s\n', filename_est);

