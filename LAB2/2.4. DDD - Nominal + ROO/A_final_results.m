%% Direct Digital Design [D³]: Nominal Tracking Case

% Define model and block names

model_name = 'Model_Direct_design_controller_SS_nominal_tracking_system';

step_ref   = 'Position reference [deg]';

% Sampling times to test
T1 = 1e-3; T2 = 10e-3; T3 = 50e-3;
Ts_values = [T1, T2, T3];

A_ref = 50;                   % step amplitude (deg)

% Preallocate results table
Results = table('Size',[numel(Ts_values) 4], ...
    'VariableTypes',{'double','double','double','double'}, ...
    'VariableNames',{'Ts','RiseTime','SettlingTime','Overshoot'});

open("Model_Direct_design_controller_SS_nominal_tracking_system.slx");

% Loop over sampling times
for i = 1:numel(Ts_values)

    Ts = Ts_values(i);
    
    % --- 1) Re-design all control variables for this Ts
    [rad2deg, deg2rad, Nu, Nx, K_stateFB, ...
     sigmaD_tf_num, sigmaD_tf_den, ...
     Phi_o, Gamma_o, H_o, J_o, Ts] = Controller_SS_direct_design_control_vars(Ts);

    % --- 2) Push all required variables to base workspace
    assignin('base','Ts', Ts);

    assignin('base','Nu', Nu);
    assignin('base','Nx', Nx);

    assignin('base','K_stateFB', K_stateFB);

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
    y = simOut.get('pos_measured').Data;
    t = simOut.get('pos_measured').Time;
    
    % --- 7) compute stepinfo
    S = stepinfo(y, t, A_ref, 'SettlingTimeThreshold',0.05);
    
    % --- 8) store
    Results(i, :) = {Ts, S.RiseTime, S.SettlingTime, S.Overshoot};
end

% Display table
disp('Direct Digital Design — Step Validation Results');
disp(Results);

% Save to CSV
csv_filename = 'results_direct_design.csv';
writetable(Results, csv_filename);
fprintf('Results saved to %s\n', csv_filename);

% Plot metrics vs Ts
figure('Name','Direct Digital Design Performance');
subplot(3,1,1);
plot(Results.Ts, Results.RiseTime, '-o','LineWidth',1.5);
ylabel('Rise Time [s]'); title('Rise Time vs Sampling Time'); grid on;

subplot(3,1,2);
plot(Results.Ts, Results.SettlingTime, '-o','LineWidth',1.5);
ylabel('Settling Time [s]'); title('Settling Time vs Sampling Time'); grid on;

subplot(3,1,3);
plot(Results.Ts, Results.Overshoot, '-o','LineWidth',1.5);
ylabel('Overshoot [%]'); xlabel('Sampling Time [s]'); title('Overshoot vs Sampling Time'); grid on;

% Save figure
fig_filename = 'plot_direct_design.png';
saveas(gcf, fig_filename);
fprintf('Plot saved to %s\n', fig_filename);

close_system("Model_Direct_design_controller_SS_nominal_tracking_system.slx");