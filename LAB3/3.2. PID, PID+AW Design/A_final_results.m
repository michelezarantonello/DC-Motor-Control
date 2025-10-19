%% 1. Load Parameters (Motor+Hub+Beam, PID Tuning)
run('Plant_ModelParameters.m');
run('Controller_PIDTuning.m');

%% 2. Open Simulink Model
model_name = 'Model_LAB3_Complete';
load_system(model_name);

%% 3. Configuration
A_set = [50, 120];              % Step references in degrees
controller_labels = ["PID", "PID + AW"];
results = table();                              % Combined results table
sim_time = 10;                                   % Simulation time in seconds

%% 4. Sweep Through References and Controller Types
for ctrl_mode = 1:2
    set_param([model_name '/PID + AW/Constant'], 'Value', num2str(ctrl_mode));

    for ref = A_set
        % Set reference step amplitude
        set_param([model_name '/Position Reference [deg]'], 'After', num2str(ref));

        % Run simulation
        simOut = sim(model_name, 'StopTime', num2str(sim_time), ...
                     'ReturnWorkspaceOutputs', 'on');

        % Extract position measurement
        pos = simOut.get('pos_meas_hub');
        t = pos.Time;
        y = pos.Data;

        % Step info
        S = stepinfo(y, t, 'RiseTimeLimits', [0.1 0.9], ...
                           'SettlingTimeThreshold', 0.05);

        % Append to result table
        results = [results;
            table(controller_labels(ctrl_mode), ref, ...
                  S.RiseTime, S.SettlingTime, S.Overshoot, ...
                  'VariableNames', {'Controller', 'RefAmplitude_deg', ...
                                    'RiseTime', 'SettlingTime', 'Overshoot'})];
    end
end

%% Tabular Display of Step Response Metrics:
% Extract unique step references
ref_vals = unique(results.RefAmplitude_deg);

% Separate tables for each controller
results_pid = results(results.Controller == "PID", :);
results_aw  = results(results.Controller == "PID + AW", :);

% Build tables (rows = references, columns = metrics)
T_pid = table(...
    results_pid.RiseTime, ...
    results_pid.SettlingTime, ...
    results_pid.Overshoot, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot'}, ...
    'RowNames', strcat(string(ref_vals), " deg"));

T_aw = table(...
    results_aw.RiseTime, ...
    results_aw.SettlingTime, ...
    results_aw.Overshoot, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot'}, ...
    'RowNames', strcat(string(ref_vals), " deg"));

% Display results
disp("=== PID Controller Step Response Metrics ===");
disp(T_pid);

disp("=== PID + AW Controller Step Response Metrics ===");
disp(T_aw);

%% 5. Save CSV
filename_csv = 'LAB3_results.csv';
writetable(results, filename_csv);
fprintf('✅ Saved step response results to: %s\n', filename_csv);


%% 6. Plot Step Response Metrics
figure('Name', 'Performance Metrics', 'Position', [100 100 1000 800]);

% Extract common x-axis (reference amplitudes)
g = results.RefAmplitude_deg(results.Controller == "PID");

% Set x-axis range with margin
x_margin = 10;
x_min = min(g) - x_margin;
x_max = max(g) + x_margin;

% --------- Rise Time ---------
subplot(3,1,1); hold on; grid on;
plot(g, results.RiseTime(results.Controller == "PID"), 'b-o', 'DisplayName', 'PID');
plot(g, results.RiseTime(results.Controller == "PID + AW"), 'r-s', 'DisplayName', 'PID + AW');
xlim([x_min, x_max]);
ylabel('Rise Time (s)');
xlabel('Reference Amplitude [deg]');
title('Rise Time vs. Reference Amplitude');
legend;

% --------- Settling Time ---------
subplot(3,1,2); hold on; grid on;
plot(g, results.SettlingTime(results.Controller == "PID"), 'b-o');
plot(g, results.SettlingTime(results.Controller == "PID + AW"), 'r-s');
xlim([x_min, x_max]);
ylabel('Settling Time (s)');
xlabel('Reference Amplitude [deg]');
title('Settling Time vs. Reference Amplitude');
legend('PID', 'PID + AW');

% --------- Overshoot ---------
subplot(3,1,3); hold on; grid on;
plot(g, results.Overshoot(results.Controller == "PID"), 'b-o');
plot(g, results.Overshoot(results.Controller == "PID + AW"), 'r-s');
xlim([x_min, x_max]);
ylabel('Overshoot (%)');
xlabel('Reference Amplitude [deg]');
title('Overshoot vs. Reference Amplitude');
legend('PID', 'PID + AW');

%% 7. Save Plot
filename_plot = 'LAB3_performance_plot.png';
saveas(gcf, filename_plot);
fprintf('✅ Saved performance plot to: %s\n', filename_plot);

close_system('Model_LAB3_Complete.slx',0);
