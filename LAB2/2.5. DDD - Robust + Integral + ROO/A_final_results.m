%% Direct Digital Design [D³]: Validation Robust+Integral Tracking Case

% Model: Direct_design_controller_SS_robust_tracking_system
% Step block: 'Position reference [deg]'
% Integrator block: 'DT-integrator' (only Ts changes)
% To Workspace signal: pos_measured
% Sampling times: T1, T2, T3 via Ts_values
% Metrics: RiseTime, SettlingTime, Overshoot
% Outputs: results_DDD_robust.csv, plot_DDD_robust.png


%% Model Parameters
model_name   = 'Model_Direct_design_controller_SS_robust_tracking_system';

step_block   = 'Position reference [deg]';

int_block    = 'DT-Integrator';

to_ws_signal = 'pos_measured';

Ts_values    = [0.001, 0.01, 0.05];   % sampling times (s)
A_ref        = 50;                   % step amplitude (deg)

ssm_block = [model_name '/DT ROO/ROO'];

%% Open model
open_system(model_name);

%% Preallocate table
Results = table('Size',[numel(Ts_values) 4], ...
    'VariableTypes',{'double','double','double','double'}, ...
    'VariableNames',{'Ts','RiseTime','SettlingTime','Overshoot'});

%% Loop through sampling times
for i = 1:numel(Ts_values)

    Ts = Ts_values(i);

    % --- 1) Re-design all control variables for this Ts
    [rad2deg, deg2rad, Nu, Nx, K_stateFB, K_I, ...
     sigmaD_tf_num, sigmaD_tf_den, ...
     Phi_o, Gamma_o, H_o, J_o, Ts] = Controller_SS_direct_design_control_vars_robusttracking(Ts);

    % --- 2) Push all required variables to base workspace
    assignin('base','Ts', Ts);

    assignin('base','Ts',Ts);
    set_param([model_name '/' int_block],'SampleTime',num2str(Ts));

    assignin('base','Nu', Nu);
    assignin('base','Nx', Nx);

    assignin('base','K_stateFB', K_stateFB);
    assignin('base','K_I', K_I);

    assignin('base','Phi_o', Phi_o);
    assignin('base','Gamma_o', Gamma_o);
    assignin('base','H_o', H_o);
    assignin('base','J_o', J_o);

    assignin('base','sigmaD_tf_num', sigmaD_tf_num);
    assignin('base','sigmaD_tf_den', sigmaD_tf_den);


    % --- 4) Refresh Simulink engine to apply all updates
    set_param(model_name, 'SimulationCommand', 'update');
    pause(0.1);

    % --- 5) simulate
    simOut = sim(model_name, ...
                 'StopTime','5', ...
                 'ReturnWorkspaceOutputs','on');
    
    % --- 6) extract measured output
    y = simOut.get(to_ws_signal).Data;
    t = simOut.get(to_ws_signal).Time;
    
    % --- 7) compute stepinfo
    S = stepinfo(y, t, A_ref, 'SettlingTimeThreshold',0.05);
    
    % --- 8) store
    Results(i, :) = {Ts, S.RiseTime, S.SettlingTime, S.Overshoot};
end

%% Display & save
disp('Direct Digital Robust Tracking — Step Validation Results');
disp(Results);

writetable(Results,'results_DDD_robust.csv');
fprintf('Results saved to results_DDD_robust.csv\n');

%% Plot
figure('Name','DDD Robust Tracking Performance');
subplot(3,1,1);
plot(Results.Ts,Results.RiseTime,'-o','LineWidth',1.5);
ylabel('Rise Time [s]'); title('Rise Time vs. Sampling Time'); grid on;

subplot(3,1,2);
plot(Results.Ts,Results.SettlingTime,'-o','LineWidth',1.5);
ylabel('Settling Time [s]'); title('Settling Time vs. Sampling Time'); grid on;

subplot(3,1,3);
plot(Results.Ts,Results.Overshoot,'-o','LineWidth',1.5);
ylabel('Overshoot [%]'); xlabel('Sampling Time [s]');
title('Overshoot vs. Sampling Time'); grid on;

saveas(gcf,'plot_DDD_robust.png');
fprintf('Plot saved to plot_DDD_robust.png\n');

close_system("Model_Direct_design_controller_SS_robust_tracking_system.slx",0);


