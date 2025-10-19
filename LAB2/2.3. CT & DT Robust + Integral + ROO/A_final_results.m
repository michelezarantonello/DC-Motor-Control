%% Task 3: Step Reference Tracking Performance Test + ROO (Robust Integral)

% This script tests the perfect step reference tracking control scheme
% under robust conditions, with integrator action & ROO, for step amplitudes of 40, 70, and 120 degrees.
% It computes rise time, settling time, and overshoot, stores results in a table,
% and plots the performance trends.

% Reference amplitudes to test (degrees)
A_set = [40, 70, 120];


run('Controller_StateSpace_Lab1.m');    % setup SSMs, tune the best feedback (Plant is also setup here)

% Preallocate result table
results = table('Size', [0, 4], ...
    'VariableTypes', {'double','double','double','double'}, ...
    'VariableNames', {'RefAmplitude_deg','RiseTime_s','SettlingTime_s','Overshoot_pct'});

% Name of Simulink model (accurate model with controller already configured)

% Open the CT model
open_system('Model_statespacesystem');

model_name = 'Model_statespacesystem';  % replace with your model name if different

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
figure('Name', 'ROO in CT - Accurate Model');

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

% Close the continuous-time model
close_system('Model_statespacesystem.slx',0);

%% Task 4: Discretization of the ROO + K Feedback using FE, BE, and Tustin

open_system('Model_DT_statespacesystem');

model_name = 'Model_DT_statespacesystem';

set_param(model_name, 'SimulationCommand', 'update');
pause(0.1);

step_ref   = 'Position reference [deg]';

% Optionally clear any lingering simOut or results
clear simOut Results;

% Sampling times
Ts_values = [0.001, 0.01, 0.05];

methods = {'FE','BE','Tustin'};

for m = 1:numel(methods)
    method = methods{m};
    
    % Map to human‐readable names for titles, file names, integrator
    switch method
      case 'FE'
        ctrlName  = 'ROO - FE';
        intMethod = 'Forward Euler';
      case 'BE'
        ctrlName  = 'ROO - BE';
        intMethod = 'Backward Euler';
      case 'Tustin'
        ctrlName  = 'ROO - Tustin';
        intMethod = 'Trapezoidal';
    end
    
    % Run the step‐test for this method
    fprintf('\n=== Testing %s discretization ===\n', ctrlName);
    ssm_50degStep(model_name, step_ref, Ts_values, ctrlName, method, intMethod, A_o, B_o, C_o, D_o);
end


function ssm_50degStep(model_name, step_ref, Ts_values, ctrlName, method, intMethod, A_o, B_o, C_o, D_o)
    % Runs a 50° step test over Ts_values for a given discretization method.

    A_ref = 50;  % step amplitude [deg]
    safeName = regexprep(ctrlName,'\s|[:\-]','_');
    
    Results = table('Size',[numel(Ts_values) 4], ...
        'VariableTypes',{'double','double','double','double'}, ...
        'VariableNames',{'Ts','RiseTime','SettlingTime','Overshoot'});

    for i = 1:numel(Ts_values)
        Ts = Ts_values(i);
        
        % 1) Build DT observer matrices for this method & Ts

        switch method
          case 'FE'
            Phi_o = eye(size(A_o)) + A_o*Ts;  
            Ksi_o = B_o * Ts;
            Ho    = C_o;
            Jo    = D_o;
      
            
          case 'BE'
            Phi_o = (eye(size(A_o)) - A_o*Ts)^(-1);
            Ksi_o = Phi_o * B_o * Ts;
            Ho    = C_o * Phi_o;
            Jo    = D_o + C_o * Phi_o * B_o * Ts;

          case 'Tustin'
            Pm    = eye(size(A_o)) - (A_o*Ts)/2;
            Pp    = eye(size(A_o)) + (A_o*Ts)/2;
            Phi_o = Pp / Pm;
            Ksi_o = (Pm^(-1)) * B_o * sqrt(Ts);
            Ho    = sqrt(Ts) * C_o * (Pm^(-1));
            Jo    = D_o + C_o * (Pm^(-1)) * (B_o * Ts/2);
  
    end
        
        % 2) Push into the DT SSM block
        set_param([model_name '/DT SSM'], ...
            'A', mat2str(Phi_o), ...
            'B', mat2str(Ksi_o), ...
            'C', mat2str(Ho), ...
            'D', mat2str(Jo));
        set_param(model_name,'SimulationCommand','update');

        % Update the integrator method once
        set_param([model_name '/SSM_Robust_Integral/Discrete-Time Integrator'], ...
                 'IntegratorMethod', intMethod);
        set_param(model_name, 'SimulationCommand','update');
        
        % 3) Simulate step response
        assignin('base','Ts',Ts);
        set_param([model_name '/' step_ref],'After',num2str(A_ref));
        set_param(model_name,'SimulationCommand','update');
        simOut = sim(model_name,'StopTime','5');
        
        y = simOut.get('pos_meas_ssm').Data;
        t = simOut.get('pos_meas_ssm').Time;
        S = stepinfo(y, t, A_ref, 'SettlingTimeThreshold',0.05);

        Results(i, :) = {Ts, S.RiseTime, S.SettlingTime, S.Overshoot};


    end
    
    % 4) Display, save table, and plot
    fprintf('Results for %s:\n', ctrlName);
    disp(Results);
    writetable(Results, sprintf('results_%s.csv', safeName));
    
    figure('Name',sprintf('%s Performance',ctrlName));
    subplot(3,1,1);
    plot(Results.Ts, Results.RiseTime, '-o','LineWidth',1.5); hold on;
    plot(Results.Ts(end), Results.RiseTime(end), 'ro','MarkerSize',8);
    title([ctrlName, ': Rise Time vs Ts']); ylabel('Rise Time [s]'); grid on;

    subplot(3,1,2);
    plot(Results.Ts, Results.SettlingTime, '-o','LineWidth',1.5); hold on;
    plot(Results.Ts(end), Results.SettlingTime(end), 'ro','MarkerSize',8);
    title([ctrlName, ': Settling Time vs Ts']); ylabel('Settling Time [s]'); grid on;

    subplot(3,1,3);
    plot(Results.Ts, Results.Overshoot, '-o','LineWidth',1.5); hold on;
    plot(Results.Ts(end), Results.Overshoot(end), 'ro','MarkerSize',8);
    title([ctrlName, ': Overshoot vs Ts']); ylabel('Overshoot [%]'); xlabel('Ts [s]'); grid on;
    
    saveas(gcf, sprintf('plot_%s.png', safeName));
end

close_system("Model_DT_statespacesystem.slx",0);

