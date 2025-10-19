%% SSM Controller (Nominal Case) using Eigenvalue Placement & LQR

% This script runs both Controller Design Methods and updates K_fb
% accordingly; it then produces the step response tables, plots, and saves
% them as per usual.

%% Setup

% Open the relevant Simulink model:
open('Model_LAB3_ssSYSTEM.slx');

% Load Motor Parameters
run("Plant_ModelParameters.m");

% Load Simulink Model
model_name = 'Model_LAB3_ssSYSTEM';
load_system(model_name);

A_set = [50];      % Step amplitude [deg]
sim_time = 10;     % seconds

controller_scripts = ["Controller_LAB3_ssVAR.m", "Controller_LAB3_LQR.m", "Controller_LAB3_LQR_Matrix.m"];
controller_labels  = ["Eigenvalue Placement", "Scalar LQR","Matrix LQR (via Bryson)"];

results = table();

%% Sweep over controller modes
for ctrl_mode = 1:3
    
    clear K_fb;                 % Ensure no previous controller is left behind
    
    run(controller_scripts(ctrl_mode));  % This sets the matrix K
    assignin('base', 'K_fb', K_fb);

    for ref = A_set

        % Force model to recompile and pick up new K_fb
        set_param(model_name, 'SimulationCommand', 'update');

        set_param([model_name '/Position Reference [deg]'], 'After', num2str(ref));

        simOut = sim(model_name, 'StopTime', num2str(sim_time), ...
                     'ReturnWorkspaceOutputs', 'on');

        pos = simOut.get('pos_meas_hub');
        t = pos.Time;
        y = pos.Data;

        S = stepinfo(y, t, 'RiseTimeLimits', [0.1 0.9], ...
                           'SettlingTimeThreshold', 0.05);

        results = [results;
            table(controller_labels(ctrl_mode), ref, ...
                  S.RiseTime, S.SettlingTime, S.Overshoot, ...
                  'VariableNames', {'Controller', 'RefAmplitude_deg', ...
                                    'RiseTime', 'SettlingTime', 'Overshoot'})];
    end
end

%% Tabular Display of Step Response Metrics

T1 = results(results.Controller == "Eigenvalue Placement", :);
T2 = results(results.Controller == "Scalar LQR", :);
T3 = results(results.Controller == "Matrix LQR (via Bryson)", :);

disp("=== Step Response Metrics: Eigenvalue Placement ===");
disp(table(T1.RiseTime, T1.SettlingTime, T1.Overshoot, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot'}, ...
    'RowNames', strcat(string(T1.RefAmplitude_deg), " deg")));

disp("=== Step Response Metrics: Scalar LQR ===");
disp(table(T2.RiseTime, T2.SettlingTime, T2.Overshoot, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot'}, ...
    'RowNames', strcat(string(T2.RefAmplitude_deg), " deg")));

disp("=== Step Response Metrics: Matrix LQR ===");
disp(table(T3.RiseTime, T3.SettlingTime, T3.Overshoot, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot'}, ...
    'RowNames', strcat(string(T3.RefAmplitude_deg), " deg")));

%% Save results
filename_csv = 'LAB3_ss_results.csv';
writetable(results, filename_csv);
fprintf('✅ Saved step response results to: %s\n', filename_csv);

%% Plotting
figure('Name', 'Step Response Metrics', 'Position', [100 100 1000 800]);

% X-axis categories for controller types
x = 1:3;
x_labels = {'Eigenvalue Placement', 'Scalar LQR', 'Matrix LQR'};

% Extract values
rise_vals      = [T1.RiseTime(1),     T2.RiseTime(1),     T3.RiseTime(1)];
settling_vals  = [T1.SettlingTime(1), T2.SettlingTime(1), T3.SettlingTime(1)];
overshoot_vals = [T1.Overshoot(1),    T2.Overshoot(1),    T3.Overshoot(1)];

% --------- Rise Time ---------
subplot(3,1,1); hold on; grid on;
plot(x, rise_vals, 'b-o', 'MarkerFaceColor','none', 'MarkerEdgeColor','b', ...
     'LineWidth', 1);
text(x, rise_vals, compose('%.2f', rise_vals), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', ...
     'FontSize', 10, 'Color', 'k');
xlim([0.5 3.5]); 
set(gca, 'XTick', x, 'XTickLabel', x_labels);
ylabel('Rise Time (s)');
title('Rise Time Comparison');

% --------- Settling Time ---------
subplot(3,1,2); hold on; grid on;
plot(x, settling_vals, 'b-s', 'MarkerFaceColor','none', 'MarkerEdgeColor','b', ...
     'LineWidth', 1);
text(x, settling_vals, compose('%.2f', settling_vals), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', ...
     'FontSize', 10, 'Color', 'k');
xlim([0.5 3.5]); 
set(gca, 'XTick', x, 'XTickLabel', x_labels);
ylabel('Settling Time (s)');
title('Settling Time Comparison');

% --------- Overshoot ---------
subplot(3,1,3); hold on; grid on;
plot(x, overshoot_vals, 'b-d', 'MarkerFaceColor','none', 'MarkerEdgeColor','b', ...
     'LineWidth', 1);
text(x, overshoot_vals, compose('%.2f', overshoot_vals), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', ...
     'FontSize', 10, 'Color', 'k');
xlim([0.5 3.5]); 
set(gca, 'XTick', x, 'XTickLabel', x_labels);
ylabel('Overshoot (%)');
title('Overshoot Comparison');


% Save plot
filename_plot = 'LAB3_performance_plot.png';
saveas(gcf, filename_plot);
fprintf('✅ Saved performance plot to: %s\n', filename_plot);

close_system('Model_LAB3_ssSYSTEM.slx',0);





