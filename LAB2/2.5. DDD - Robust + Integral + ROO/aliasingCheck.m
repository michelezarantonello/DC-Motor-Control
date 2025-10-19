function aliasingCheck(Ts, ssm_model)

    % Step 1: Loop transfer function as num(T(s)) = C(s)*P(s)/[1+C(s)*P(s)]

    sigmaD_tf = tf(ssm_model);
    
    %turn the SSM into a TF
    [sigmaD_tf_num, ~] = tfdata(sigmaD_tf, 'v');

    % Step 2: Nyquist frequency
    omega_N = pi / Ts; % rad/s
    fprintf('\nNyquist frequency ω_N = %.2f rad/s\n', omega_N);

    % Step 3: Generate Bode data
    [mag, phase, w] = bode(sigmaD_tf_num, omega_N);
    mag = squeeze(mag);  % squeeze to get vector
    phase = squeeze(phase);
    w = squeeze(w);      % [rad/s]

    % Step 4: Normalize gain (peak = 0 dB) to define "roll-off"
    mag_db = 20*log10(mag);
    peak_db = max(mag_db);
    mag_db = mag_db - peak_db;

    % Step 5: Find roll-off frequency (e.g., where |L(jw)| drops below −3 dB)
    if nargin < 4
        threshold_db = -3;  % default roll-off threshold
    end
    idx_rolloff = find(mag_db <= threshold_db, 1);  % first crossing

    if isempty(idx_rolloff)
        warning('Loop gain never falls below %.1f dB – treating as broadband.', threshold_db);
        omega_B = w(end);  % use highest frequency tested
    else
        omega_B = w(idx_rolloff);
    end

    % Step 6: Report
    fprintf('Bandwidth ω_B (%.1f dB roll-off): %.2f rad/s\n', threshold_db, omega_B);

    % Step 7: Compare against Nyquist frequency
    if omega_B >= 0.7 * omega_N
        warning('⚠️ Potential aliasing: ω_B (%.2f) is close to or exceeds ω_N/2 (%.2f)\n', omega_B, 0.5*omega_N);
    else
        fprintf('✅ No aliasing concern: ω_B below 70 percent of Nyquist limit.\n');
    end

    % Optional: Plot for confirmation
    figure;
    bode(sigmaD_tf_num, omega_N); grid on;
    title('Loop Transfer Function Bode Plot');
    hold on;
    yline(threshold_db + peak_db, 'r--', 'Threshold');
    xline(omega_B, 'g--', sprintf('ω_B ≈ %.2f rad/s', omega_B));
    xline(omega_N, 'k-.', sprintf('ω_N = %.2f rad/s', omega_N));
end