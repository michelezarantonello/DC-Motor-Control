%% Step Reference Tracking Performance Test (Robust Integral)

% This script tests the perfect step reference tracking control scheme
% under robust conditions, with integrator action, for step amplitudes of 40, 70, and 120 degrees.
% It computes rise time, settling time, and overshoot, stores results in a table,
% and plots the performance trends.

% Reference amplitudes to test (degrees)
A_set = [40, 70, 120];


run('Controller_StateSpace_Lab1.m');             % setup SSMs, tune the best feedback

% Preallocate result table
results = table('Size', [0, 4], ...
    'VariableTypes', {'double','double','double','double'}, ...
    'VariableNames', {'RefAmplitude_deg','RiseTime_s','SettlingTime_s','Overshoot_pct'});

model_name = 'Model_statespacesystem';  % replace with your model name if different

open("Model_statespacesystem.slx");

set_param(model_name, 'SimulationCommand', 'update');
pause(0.1);

% Loop over each reference amplitude
for a = A_set
    % Set the step block amplitude (in degrees)
    set_param([model_name '/Position reference [deg]'], 'After', num2str(a));

    % Run the simulation and return workspace outputs
    simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');

    % Extract the measured position response (accurate model)
    pos_acc = simOut.get('pos_meas_ssm');  % pos_meas_ssm is a Dataset
    t_acc = pos_acc.Time;                  % Time is a vector
    y_acc = pos_acc.Data;                  % Data is a vector

    % Ensure both t_acc and y_acc are column vectors for stepinfo
    t_acc = t_acc(:);  % Ensure it's a column vector
    y_acc = y_acc(:);  % Ensure it's a column vector
    
    % Compute performance metrics using stepinfo
    info = stepinfo(y_acc, t_acc, ...
        'RiseTimeLimits', [0.1 0.9], ...    % 10%–90% rise time
        'SettlingTimeThreshold', 0.05);     % 5% settling threshold

    % Append results to table
    results = [results; 
        table(a, info.RiseTime, info.SettlingTime, info.Overshoot, ...
              'VariableNames', results.Properties.VariableNames)];
end

% Display results in Command Window
disp('Step Tracking Performance (Accurate Model)');
disp(results);

% Save results to CSV file
outFile = 'Results_Appx_acc_ssm_Model.csv';
writetable(results, outFile);
fprintf('Results saved to %s\n', outFile);

% Plot performance trends
figure('Name', 'Performance Trends - Accurate Model');

subplot(3,1,1);
plot(results.RefAmplitude_deg, results.RiseTime_s, '-o', 'LineWidth', 1.5);
ylabel('Rise Time (s)');
title('Rise Time vs. Reference Amplitude');
grid on;

subplot(3,1,2);
plot(results.RefAmplitude_deg, results.SettlingTime_s, '-o', 'LineWidth', 1.5);
ylabel('Settling Time (s)');
title('Settling Time vs. Reference Amplitude');
grid on;

subplot(3,1,3);
plot(results.RefAmplitude_deg, results.Overshoot_pct, '-o', 'LineWidth', 1.5);
ylabel('Overshoot (%)');
xlabel('Reference Amplitude (deg)');
title('Overshoot vs. Reference Amplitude');
grid on;

% Save plot as image
saveas(gcf, fullfile(pwd, 'performance_trends_accurate.png'));

close_system("Model_statespacesystem.slx",0);
