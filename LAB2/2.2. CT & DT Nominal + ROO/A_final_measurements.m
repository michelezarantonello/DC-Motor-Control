%% Reduced - Order Observer Testing (Nominal)
% This script tests the perfect step reference tracking control scheme
% under nominal conditions for step amplitudes of 40, 70, and 120 degrees.
% It computes rise time, settling time, and overshoot, stores results in a table,
% and plots the performance trends.

%  Load motor Parameters
run("Plant_EstimTrialUpdateLAB17_04_25.m");

%  Setup Observer Model
run("Controller_StateSpaceDigitalController.m");

%% Task 1: CT Model Amplitude Sweep

% Reference amplitudes to test (degrees)
A_set = [40, 70, 120];


% Preallocate result table
results = table('Size', [0, 4], ...
    'VariableTypes', {'double','double','double','double'}, ...
    'VariableNames', {'RefAmplitude_deg','RiseTime_s','SettlingTime_s','Overshoot_pct'});

% Open the CT model
open_system('Model_SSReducedOrderNominalTrackingSystem.slx');


% Name of Simulink model (accurate model with controller already configured)
model_name = 'Model_SSReducedOrderNominalTrackingSystem';  % replace with your model name if different

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

% Close the continuous-time model
close_system('Model_SSReducedOrderNominalTrackingSystem.slx',0);

%% Task 2: DT Nominal Tracking Performance — FE, BE, Tustin  

open_system('Model_DiscreteSSRedOrderNominalTrackingSystem.slx');

model_name = 'Model_DiscreteSSRedOrderNominalTrackingSystem';

set_param(model_name, 'SimulationCommand', 'update');
pause(0.1);

ssm_block  = [model_name '/Discrete State-Space'];

step_ref   = 'Position reference [deg]';

Ts_values = [0.001, 0.01, 0.05];

methods   = {'FE','BE','Tustin'};

for m = 1:numel(methods)
  method = methods{m};
  switch method
    case 'FE'
      ctrlName = 'Nominal - FE';
    case 'BE'
      ctrlName = 'Nominal - BE';
    case 'Tustin'
      ctrlName = 'Nominal - Tustin';
  end
  safeName = regexprep(ctrlName,'\s|[:\-]','_');
  
  % Preallocate
  Results = table('Size',[numel(Ts_values) 4], ...
      'VariableTypes',{'double','double','double','double'}, ...
      'VariableNames',{'Ts','RiseTime','SettlingTime','Overshoot'});
  
  fprintf('\n=== Testing %s discretization ===\n', ctrlName);
  
  for i = 1:numel(Ts_values)

    Ts = Ts_values(i);
    
    % Build DT matrices from CT A_o,B_o,C_o,D_o
    switch method
      case 'FE'
        A_d = eye(size(A_o)) + A_o*Ts; 
        B_d = B_o*Ts;
        C_d = C_o;
        D_d = D_o;
        
      case 'BE'
        A_d = (eye(size(A_o)) - A_o*Ts)^(-1);
        B_d = A_d * B_o * Ts;
        C_d = C_o * A_d;
        D_d = D_o + C_o * A_d * B_o * Ts;
        
      case 'Tustin'
        Pm = eye(size(A_o)) - (A_o*Ts)/2;
        Pp = eye(size(A_o)) + (A_o*Ts)/2;
        A_d = Pp / Pm;
        B_d = (Pm^(-1)) * B_o * sqrt(Ts);
        C_d = sqrt(Ts) * C_o * (Pm^(-1));
        D_d = D_o + C_o * (Pm^(-1)) * (B_o * Ts/2);
    end

    
    % Push into Discrete State-Space block
    set_param(ssm_block, ...
      'A', mat2str(A_d), ...
      'B', mat2str(B_d), ...
      'C', mat2str(C_d), ...
      'D', mat2str(D_d));

    set_param(model_name,'SimulationCommand','update');
    
    % Simulate 50° step
    assignin('base','Ts',Ts);
    set_param([model_name '/' step_ref],'After',num2str(50));
    set_param(model_name,'SimulationCommand','update');
    simOut = sim(model_name,'StopTime','5');
    
    % Extract response
    y = simOut.get('pos_measured').Data;
    t = simOut.get('pos_measured').Time;
    S = stepinfo(y, t, 50, 'SettlingTimeThreshold',0.05);
    
    Results(i, :) = {Ts, S.RiseTime, S.SettlingTime, S.Overshoot};
  end
  
  % Display & save
  fprintf('Results for %s:\n', ctrlName);
  disp(Results);
  writetable(Results, sprintf('results_%s.csv', safeName));
  
  % Plot
  figure('Name',sprintf('%s Performance',ctrlName));
  subplot(3,1,1);
  plot(Results.Ts,Results.RiseTime,'-o','LineWidth',1.5); hold on;
  plot(Results.Ts(end),Results.RiseTime(end),'ro','LineWidth',1.5);
  title([ctrlName,': Rise Time']); ylabel('Rise Time [s]'); grid on;
  
  subplot(3,1,2);
  plot(Results.Ts,Results.SettlingTime,'-o','LineWidth',1.5); hold on;
  plot(Results.Ts(end),Results.SettlingTime(end),'ro','LineWidth',1.5);
  title([ctrlName,': Settling Time']); ylabel('Settling Time [s]'); grid on;
  
  subplot(3,1,3);
  plot(Results.Ts,Results.Overshoot,'-o','LineWidth',1.5); hold on;
  plot(Results.Ts(end),Results.Overshoot(end),'ro','LineWidth',1.5);
  title([ctrlName,': Overshoot']); ylabel('Overshoot [%]'); xlabel('Ts [s]'); grid on;
  
  saveas(gcf, sprintf('plot_%s.png', safeName));
end

close_system("Model_DiscreteSSRedOrderNominalTrackingSystem.slx",0);