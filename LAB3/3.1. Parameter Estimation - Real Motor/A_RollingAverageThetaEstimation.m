%% 1) Nominal/tentative parameters (from Table 1 & Listing 1)

Jb    = 1.4e-3;        % beam inertia [kg·m^2]

Bb_nom    = 3.4e-3;    %  tentative beam viscous friction [N·m/(rad/s)]
k_nom     = 0.83;      %  tentative joint stiffness [N·m/rad]

deg2rad = pi/180;


%% 1) Simulation Readout

load("BeamOsc10s_4018msTO7000ms_16_5deg.mat"); %.mat file containing beam displacement over time

T = THETA_DIFFERENCE.signals.values;
 
fs = 1000;  % sampling frequency [1kHz]
Ts = 1/fs;  % sampling period    [1ms]

new_signal = [];

Osc_start_amp = 16.5 * deg2rad; % [rad]

Osc_start_ms = 4810; % starting time in [ms]
Osc_end_ms = 6836-245;   % ending time in   [ms]

% defining the relevant interval we analyze
n_samples = Osc_end_ms - Osc_start_ms;

i = 1;

% extracting the signal over the interval of interest
while i + Osc_start_ms <= Osc_end_ms

    new_signal(i) = T(i + Osc_start_ms);

    i = i + 1;
end

% if length(new_signal(1,:)) == n_samples
%     display('Interval Extracted Correctly!');
% else
%     display("Check intervals again!");
% end

%% 2) Rolling Average (tradeoff between signal smoothness & objective value): 
% a higher window size makes for a smoother signal but due to the averaging
% function, this shkrinks the signal size, losing its original magnitude

w = 12;                     

n = length(new_signal);

rolling_avg = zeros(1, n - w + 1);  % Preallocate output vector

% computing the rolling average
for i = 1:(n - w + 1)
    rolling_avg(i) = mean(new_signal(i:i+w-1));
end

t = (0:n-1) *Ts; 
tt = (0:length(rolling_avg)-1) *Ts;

new_signal = new_signal *deg2rad;
rolling_avg = rolling_avg * deg2rad;

%% Plot: Raw vs. Smoothed (Rolling Avg) Signal

figure;
plot(t, new_signal, 'r--', 'DisplayName','Raw signal');
hold on;
plot(tt, rolling_avg, 'b', 'LineWidth', 0.5, 'DisplayName','Rolling avg');
grid on;
xlabel('Time (s)');
ylabel('\theta_d [rad]');
title('Raw signal vs. Rolling Average');
legend('Location','northeast');

save('SmoothThetaSignal.mat', 'rolling_avg');

%% 3) Extract M peaks of |θ(t)| to build data {k, log|θ(t_k)|}

M = 13;  % number of peaks to use (the remaining 3 peaks are very noisy)

% find local maxima of abs(rolling_avg), returning their SAMPLE INDICES
[pk_vals, locs] = findpeaks(abs(rolling_avg), 'NPeaks', M, 'SortStr','descend','MinPeakDistance',125);

% convert indices to times in seconds
tk = (locs - 1) * Ts;    % now tk is strictly in seconds

% sort by time (in case findpeaks returned them out of order)
[tk, ix]   = sort(tk);
pk_vals    = pk_vals(ix);

% build regression data
k_idx = (0:M-1).';               % 0,1,2,...,M-1
Y     = log(pk_vals)';            % log |θ(t_k)|

Phi   = [-k_idx, ones(M,1)];     % z = -ξ*k + b

%% 4) LS fit for logarithmic decrement ξ

theta_ls = (Phi' * Phi)^(-1) * (Phi' * Y); 

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

A_env = max(abs(rolling_avg));      % estimated amplitude envelope A

env_pos = A_env * exp(sigma_hat * tt);  % upper envelope
env_neg = -env_pos;                 % lower envelope

figure;
subplot(2,1,1)
plot(tt, rolling_avg, 'b'); hold on
plot(tt, env_pos, '--r', 'DisplayName','Envelope');
plot(tt, env_neg, '--r', 'HandleVisibility','off');
ylabel('θ_d [rad]'), grid on
title('Free oscillation with damping envelope');
legend('θ_d(t)', 'Damping envelope');

subplot(2,1,2)
plot(tt, abs(rolling_avg), 'k');
ylabel('|θ_d| [rad]'), xlabel('t (s)'), grid on
title('Absolute value (used for peak detection)');

%% 7.2) Compare envelope to true peak amplitudes (RMSE metric)

% Recompute estimated envelope at peak times
A_hat = pk_vals(1);  % estimated initial amplitude

% Align envelope to the first peak time ( the findpeak function otherwise
% misses the first true peak - at t0 = 0 for a cosine signal )
tk_rel = tk - tk(1);  % shift time axis so t0 = 0
env_est = A_hat * exp(sigma_hat * tk_rel);  % properly aligned


% Create vectors
true_params = [Bb_nom; k_nom];
est_params  = [Bb_est; k_est];

% Compute RMSE
param_rmse = sqrt(mean((est_params - true_params).^2));

% Optional: normalized RMSE in percent (relative to nominal magnitudes)
norm_rmse_percent = sqrt(mean(((est_params - true_params)./true_params).^2)) * 100;

% Display
fprintf('\nParameter estimation RMSE: %.4e\n', param_rmse);
fprintf('Normalized RMSE: %.2f %%\n', norm_rmse_percent);

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
peak_table = table((1:M), tk, pk_vals, log(pk_vals), ...
    'VariableNames', {'PeakIndex','Time_s','ThetaAbs_rad','LogThetaAbs'});

% Add estimated parameters as a second table row
est_table = table(delta, omega_d, omega_n, Bb_est, k_est, ...
    'VariableNames', {'delta','omega_d','omega_n','B_b_est','k_est'});

% Create filename with timestamp
filename_peaks = ['peak_data.csv'];
filename_est   = ['estimates.csv'];

% Save to CSV
writetable(peak_table, filename_peaks);
writetable(est_table, filename_est);

fprintf('Saved peak data to    : %s\n', filename_peaks);
fprintf('Saved estimates to    : %s\n', filename_est);

