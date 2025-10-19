%% Frequency-Shaped LQR (Nominal Case)
% Assign a spectral cost to the frequency at which the Natural Evolution of
% the System might undergo resonance. Function built on a separate script.

% Load Motor and SSM Parameters
run("Plant_ModelParameters.m");
run("Controller_LAB3_ssVAR.m");

open("Model_LAB3_ssSYSTEMfslqr.slx");

%% State Feedback Design using Matrix-Valued, FS-LQR

q2_vals = [0.01, 1, 100];

% Arrays to store e.m. comparison results
real_vals = zeros(size(q2_vals));
imag_vals = zeros(size(q2_vals));
attenuation_vals = zeros(size(q2_vals));

function [A_aug, B_aug, omega_0, K_fb, eig_A] = fs_lqr(q2_value, A_prime, B_prime, C_prime, D_prime)

    clear K_fb; % make way for feedback to be designed

    % Approximated system  (prepare for the SRL)
    sysG = ss(A_prime, B_prime, C_prime, D_prime);
    
    %% Obtaining the resonant frequency of the Natural System Evolution:
    
    eig_A = eigs(A_prime);
    
    omega_0 = abs(imag(eig_A(find(imag(eig_A) ~= 0, 1))));
    
    %% Building the Frequency-Shaped Components (See Assignment Paper)
    
    % Define the frequency-shaping filter
    
    H = sqrt(q2_value)*tf([0 0 (omega_0)^2], [1 0 (omega_0)^2]);

    % SSM Realization of H:

    Hss.A = [0 1; -omega_0^(2), 0];
    Hss.B = [0;1];
    Hss.C = [sqrt(q2_value)*omega_0^(2), 0];
    Hss.D = 0;

    A_q = Hss.A;
    B_q = [zeros(2,1), Hss.B, zeros(2)];
    C_q = [zeros(1,2); Hss.C; zeros(2)];

    D_q_vec = [1/((0.3)*(5)), Hss.D, 0, 0];
    D_q = diag(D_q_vec);

    % Augment A_prime and B_prime
    A_aug = [A_prime, zeros(4,2) ; B_q, A_q];
    B_aug = [B_prime; 0;0];
    C_aug = [C_prime, zeros(1,2)];
    D_aug = 0;

    Q_aug = [D_q'*D_q, D_q'*C_q; C_q'*D_q, C_q'*C_q];

    %% Solving for the Nominal Feedforward Gains:
    M = [A_aug,B_aug;C_aug,D_aug];

    Sol = M \ [0;0;0;0;0;0;1];

    Nx = Sol(1:6, :);
    Nu = Sol(7,:);

    R_aug = 1 / (10^2);  % Still bound input by 10V

    K_fb = lqr(A_aug, B_aug, Q_aug, R_aug);

    assignin('base', 'A_q', A_q);
    assignin('base', 'B_q', B_q);
    assignin('base', 'C_q', C_q);
    assignin('base', 'D_q', D_q);


    assignin('base', 'Nx', Nx);
    assignin('base', 'Nu', Nu);
    assignin('base', 'K_fb', K_fb);

end


model_name = 'Model_LAB3_ssSYSTEMfslqr';
load_system(model_name);

% Step amplitude and simulation time
A_set = [50]; % deg
sim_time = 10;

% Initialize results table
results_fs = table();

% Sweep over q2 values
for i = 1:length(q2_vals)

    q2 = q2_vals(i);

    % Design FS-LQR controller for this q2
    [A_aug, B_aug, omega_0, K_fb, eig_A] = fs_lqr(q2, A_prime, B_prime, C_prime, D_prime);

    % Eigenvalue analysis
    eig_cl = eig(A_aug - B_aug * K_fb);

    % Find closed-loop resonant eigenvalue (imag ≈ omega_0)
    [~, idx_res] = min(abs(imag(eig_cl) - omega_0));
    lambda_res = eig_cl(idx_res);

    % Find open-loop resonant eigenvalue (imag ≈ omega_0)
    [~, idx_open] = min(abs(imag(eig_A) - omega_0));
    lambda_res_open = eig_A(idx_open);

    % Store results
    real_vals(i) = real(lambda_res);
    imag_vals(i) = imag(lambda_res);
    attenuation_vals(i) = real(lambda_res) - real(lambda_res_open);

    % Update reference input in model
    set_param([model_name '/Position Reference [deg]'], 'After', num2str(A_set));

    % Simulate
    simOut = sim(model_name, 'StopTime', num2str(sim_time), ...
                             'ReturnWorkspaceOutputs', 'on');

    % Extract output
    pos = simOut.get('pos_meas_hub');
    t = pos.Time;
    y = pos.Data;

    % Evaluate step response
    S = stepinfo(y, t, 'RiseTimeLimits', [0.1 0.9], ...
                       'SettlingTimeThreshold', 0.05);

    % Store result
    results_fs = [results_fs;
        table(q2, A_set(1), S.RiseTime, S.SettlingTime, S.Overshoot, ...
              'VariableNames', {'q2_Value', 'RefAmplitude_deg', ...
                                'RiseTime', 'SettlingTime', 'Overshoot'})];
end


%% Plotting Real, Imaginary Parts and Attenuation vs. q2

figure('Name', 'Resonant Mode Shift vs q2', 'Position', [100 100 1000 700]);

% Convert q2_vals to artificial x-axis for spacing
x = 1:length(q2_vals);
x_labels = "q_{22} = " + string(q2_vals);

% Real Part (Damping)
subplot(3,1,1); hold on; grid on;
plot(x, real_vals, 'rs-', 'LineWidth', 1.5);
for i = 1:length(x)
    text(x(i)+0.05, real_vals(i), sprintf('%.2f', real_vals(i)), 'FontSize', 10);
end
ylabel('Real Part (Damping)');
title('{Re}: Resonant Pole');
set(gca, 'XTick', x, 'XTickLabel', x_labels);

% Imaginary Part (Frequency)
subplot(3,1,2); hold on; grid on;
plot(x, imag_vals, 'rs-', 'LineWidth', 1.5);
for i = 1:length(x)
    text(x(i)+0.05, imag_vals(i), sprintf('%.2f', imag_vals(i)), 'FontSize', 10);
end
ylabel('Imaginary Part (Frequency)');
title('{Im}: Closed-Loop Resonant Pole');
set(gca, 'XTick', x, 'XTickLabel', x_labels);

% Attenuation
subplot(3,1,3); hold on; grid on;
h1 = plot(x, attenuation_vals, 'ko-', 'MarkerFaceColor','none', 'LineWidth', 1.5);
plot(x, attenuation_vals, 'ko-', 'LineWidth', 1.5);
for i = 1:length(x)
    text(x(i)+0.05, attenuation_vals(i), sprintf('%.2f', attenuation_vals(i)), 'FontSize', 10);
end
ylabel('Real Axis shift');
xlabel('q_{22} Weight on θ_d');
title('Attenuation of the Resonant Mode');
set(gca, 'XTick', x, 'XTickLabel', x_labels);
% Create dummy plot handles for legend explanation
h2 = h1;
% Create dummy invisible plots for legend
h_legend1 = plot(nan, nan, 'ko-', 'DisplayName', 'Shift < 0 → Damped Resonant Mode');
h_legend2 = plot(nan, nan, 'ko-', 'MarkerFaceColor', 'k', ...
                          'DisplayName', 'Shift > 0 → Amplified Resonant Mode');

% Add legend with correct symbols and formatting
legend([h_legend1 h_legend2], ...
       'Location', 'northeast', ...
       'Interpreter', 'none');

% Save the figure
saveas(gcf, 'LAB3_FS_LQR_resonant_shift.png');
fprintf("✅ Saved figure to LAB3_FS_LQR_resonant_shift.png\n");

% Display Table
disp("=== FS-LQR Step Response Metrics ===");
disp(results_fs);

% Save table to CSV
filename_csv = 'LAB3_fslqr_results.csv';
writetable(results_fs, filename_csv);
fprintf('✅ Saved FS-LQR results to: %s\n', filename_csv);

%% Plotting Step Response

figure('Name', 'FS-LQR Performance vs q2', 'Position', [100 100 1000 800]);

% Define artificial x-axis positions for q2_vals
x = 1:3;
x_labels = {'q_2 = 0.01', 'q_2 = 1', 'q_2 = 100'};

% Rise Time
subplot(3,1,1); hold on; grid on;
plot(x, results_fs.RiseTime, 'o-', 'LineWidth', 1.5);
set(gca, 'XTick', x, 'XTickLabel', x_labels);
ylabel('Rise Time (s)');
title('Rise Time vs q_2');

% Settling Time
subplot(3,1,2); hold on; grid on;
plot(x, results_fs.SettlingTime, 's-', 'LineWidth', 1.5);
set(gca, 'XTick', x, 'XTickLabel', x_labels);
ylabel('Settling Time (s)');
title('Settling Time vs q_2');

% Overshoot
subplot(3,1,3); hold on; grid on;
plot(x, results_fs.Overshoot, 'd-', 'LineWidth', 1.5);
set(gca, 'XTick', x, 'XTickLabel', x_labels);
ylabel('Overshoot (%)');
xlabel('q_2 value');
title('Overshoot vs q_2');

% Save plot
filename_plot = 'LAB3_fslqr_performance_plot.png';
saveas(gcf, filename_plot);
fprintf('✅ Saved FS-LQR performance plot to: %s\n', filename_plot);

close_system("Model_LAB3_ssSYSTEMfslqr.slx",0);
