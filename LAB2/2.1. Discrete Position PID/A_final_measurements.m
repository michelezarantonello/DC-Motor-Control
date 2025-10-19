%% Load motor parameters
run('Plant_EstimTrialUpdateLAB17_04_25.m'); % defines Tm, km, N, etc.

%% Load parameters for Discretization
run("Controller_Discrete_Control_variables_25_04.m");

% Parameters for task execution:
model_name = 'Model_Blackbox_system_discrete_PIDs';
controller_switch = 'Controller';
step_ref = 'Position reference [deg]';


open("Model_Blackbox_system_discrete_PIDs.slx");
set_param(model_name, 'SimulationCommand', 'update');
pause(0.1);
 
%% Task 1 & 4: PID with Backward Euler, Forward Euler and Tustin 

function PID_50degStep(model_name, controller_switch, step_ref, Ts_values, val)


    % Map numeric code to human‑readable name
    switch val
      case 1, ctrlName = 'PID_EB';
      case 3, ctrlName = 'PID_EF';
      case 5, ctrlName = 'PID_Tustin';
      otherwise, ctrlName = ['Unknown(' num2str(val) ')'];
    end

    fprintf('\n=== Running %s (Controller = %d) with 50° step \n', ...
            ctrlName, val);

    % Step reference (degrees)
    A_ref = 50;
    
    % Preallocate result table
    Results = table('Size',[0 4], ...
        'VariableTypes',{'double','double','double','double'}, ...
        'VariableNames',{'Ts','RiseTime','SettlingTime','Overshoot'});
    
    % Simulation loop
    for i = 1:length(Ts_values)
        
        Ts = Ts_values(i);
    
        
        % Set sampling time
        assignin('base', 'Ts', Ts);                          
        
        % Set to PID_EB or PID_FF_EB
        set_param([model_name '/' controller_switch], 'Value',num2str(val));                
        
        % Set to 50 deg step
        set_param([model_name '/' step_ref], 'After', num2str(A_ref));                
        
        % Manual system update
        set_param(model_name, 'SimulationCommand', 'update');  
    
        % Run the Simulation
        simOut = sim(model_name, 'StopTime', '5');
        % Get measured position (assumes signal named 'pos_measured')
        y = simOut.get('pos_measured').Data;
        t = simOut.get('pos_measured').Time;
    
        % Extract performance metrics using stepinfo
        S = stepinfo(y, t, A_ref, 'SettlingTimeThreshold', 0.05); 
    
    
        % Append results to table
        Results(i, :) = {Ts, S.RiseTime, S.SettlingTime, S.Overshoot};
    
    end
    
    % Display results
    disp(Results)
    
        % Plot performance vs sampling time with poor results highlighted
    figure;

    % Indices corresponding to T2 and T3 (assuming Ts_values = [T1, T2, T3])
    bad_idx = [2 3];

    subplot(3,1,1);
    plot(Results.Ts, Results.RiseTime, '-o','LineWidth',1.5); hold on;
    plot(Results.Ts(bad_idx), Results.RiseTime(bad_idx), 'ro','MarkerSize',8,'LineWidth',1.5);
    title(sprintf('%s: Rise Time vs Sampling Time', ctrlName), 'Interpreter','none');
    ylabel('Rise Time [s]'); grid on;

    subplot(3,1,2);
    plot(Results.Ts, Results.SettlingTime, '-o','LineWidth',1.5); hold on;
    plot(Results.Ts(bad_idx), Results.SettlingTime(bad_idx), 'ro','MarkerSize',8,'LineWidth',1.5);
    title(sprintf('%s: Settling Time vs Sampling Time', ctrlName), 'Interpreter','none');
    ylabel('Settling Time [s]'); grid on;

    subplot(3,1,3);
    plot(Results.Ts, Results.Overshoot, '-o','LineWidth',1.5); hold on;
    plot(Results.Ts(bad_idx), Results.Overshoot(bad_idx), 'ro','MarkerSize',8,'LineWidth',1.5);
    title(sprintf('%s: Overshoot vs Sampling Time', ctrlName), 'Interpreter','none');
    ylabel('Overshoot [%]'); xlabel('Sampling Time [s]'); grid on;

    
    % -------------------------------------------------------------------------
    % As sampling time increases (T2, T3), closed-loop performance worsens.
    % This is because between samples, the control input from the digital PID 
    % is held constant by the ZOH (zero-order hold). During this time, the 
    % plant evolves naturally without new feedback — effectively behaving as an 
    % open-loop system. The longer the sampling interval, the more time the 
    % plant spends in open-loop evolution, leading to increased overshoot, 
    % longer settling times, and degraded control accuracy.
    % -------------------------------------------------------------------------


    % ---------------- Save table as CSV ----------------
    csv_filename = sprintf('results_%s.csv', ctrlName);
    writetable(Results, csv_filename);
    fprintf('Table saved to: %s\n', csv_filename);

    % ---------------- Save figure as PNG ----------------
    fig_filename = sprintf('plot_%s.png', ctrlName);
    saveas(gcf, fig_filename);
    fprintf('Plot saved to: %s\n', fig_filename);
    

end

%%  Task 1: Backward Euler: Test at T1, T2, T3 with 50° Step Reference
PID_50degStep(model_name, controller_switch, step_ref, Ts_values, 1);

%%  Task 4: Forward Euler:  Test at T1, T2, T3 with 50° Step Reference
PID_50degStep(model_name, controller_switch, step_ref, Ts_values, 3);

%%  Task 4: Tustin:         Test at T1, T2, T3 with 50° Step Reference
PID_50degStep(model_name, controller_switch, step_ref, Ts_values, 5);



%% Task 2: Compare Tracking Error + Step Metrics With/Without Anti‑Windup
% ------------------------------------------------------------------------
% Controllers: 
%   – Without AW: Controller = 1 (Backward Euler PID) 
%   – With    AW: Controller = 2 (Backward Euler PID + anti‑windup)
% Sampling time: T2
% Reference: 360 deg step
% Metrics: 
%   - Steady‑state tracking error (pos_error(end))
%   - Rise Time, Settling Time, Overshoot from stepinfo()
% ------------------------------------------------------------------------

% assume T2 is defined
Ts = Ts_values(2);                                
assignin('base','Ts',Ts);               % set sample time
set_param(model_name,'SimulationCommand','update');

% Reference step amplitude (deg)
A_ref = 360;
set_param([model_name '/' step_ref],'After',num2str(A_ref));

% Initialize result structures
ERR = struct('NoAW',[],'WithAW',[]);
Perf = struct('NoAW',[],'WithAW',[]);

% ----- 1) PID with Backward Euler (No AW) -----
set_param([model_name '/' controller_switch],'Value','1');
set_param(model_name,'SimulationCommand','update');

simOut = sim(model_name,'StopTime','5');

% Extract signals
e_noaw = simOut.get('pos_error').Data;
y_noaw = simOut.get('pos_measured').Data;
t_noaw = simOut.get('pos_measured').Time;

ERR.NoAW = e_noaw(end);  % steady-state error

% Get step performance metrics
S1 = stepinfo(y_noaw, t_noaw, A_ref, 'SettlingTimeThreshold', 0.05);
Perf.NoAW = S1;

% ----- 2) PID with Backward Euler + Anti-Windup -----
set_param([model_name '/' controller_switch],'Value','2');
set_param(model_name,'SimulationCommand','update');

simOut = sim(model_name,'StopTime','5');

% Extract signals
e_aw = simOut.get('pos_error').Data;
y_aw = simOut.get('pos_measured').Data;
t_aw = simOut.get('pos_measured').Time;

ERR.WithAW = e_aw(end);

% Get step performance metrics
S2 = stepinfo(y_aw, t_aw, A_ref, 'SettlingTimeThreshold', 0.05);
Perf.WithAW = S2;

% Display steady-state errors
fprintf('\nSteady-state error @ T = %.3f s, 360° step:\n', Ts);
fprintf('  Without AW: %8.4f deg\n', ERR.NoAW);
fprintf('  With    AW: %8.4f deg\n', ERR.WithAW);

% Build summary table
Metrics = table(...
    [ERR.NoAW; ERR.WithAW], ...
    [Perf.NoAW.RiseTime; Perf.WithAW.RiseTime], ...
    [Perf.NoAW.SettlingTime; Perf.WithAW.SettlingTime], ...
    [Perf.NoAW.Overshoot; Perf.WithAW.Overshoot], ...
    'VariableNames', {'SteadyStateError','RiseTime','SettlingTime','Overshoot'}, ...
    'RowNames', {'NoAW','WithAW'});

disp(Metrics);

% Save metrics table
csv_filename = 'antiwindup_comparison.csv';
writetable(Metrics, csv_filename, 'WriteRowNames', true);
fprintf('Metrics table saved to: %s\n', csv_filename);

% Optional: plot and save step responses (zoomed on transient)
figure;
plot(t_noaw, y_noaw, 'r--', 'LineWidth', 1.5); hold on;
plot(t_aw,   y_aw,   'b-',  'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Output Position (deg)');
title(sprintf('Step Response Comparison @ Ts = %.3f s, 360° step', Ts));
legend({'Without AW','With AW'}, 'Location', 'southeast');
grid on;
xlim([1 2]);  % Zoom into the transient region (1s to 2s)

% Save plot
fig_filename = 'antiwindup_step_response_zoomed.png';
saveas(gcf, fig_filename);
fprintf('Zoomed step response plot saved to: %s\n', fig_filename);

%% Task 3: Compare PID EB + Feedforward vs PID EB alone (Controller = 6)
% ------------------------------------------------------------------------
% Outer switch “Controller” = 5 selects the discrete_PID_EB_FF subsystem
% Inner switch "Constant" inside that subsystem:
%   – 1 = feedforward ON
%   – 2 = feedforward OFF (pure PID_EB)
% Sampling time to be used: T2
% Reference: generated by “FF Reference Interface” (sinusoidal position, etc.)
% Metric: RMS tracking error over simulation
% ------------------------------------------------------------------------


Ts = Ts_values(2);                         % use T2 as sampling time
assignin('base','Ts',Ts);
set_param(model_name,'SimulationCommand','update');

% select “PID_EB + FF” subsystem via outer switch = 6
set_param([model_name '/' controller_switch],'Value','6');

% inner flag path inside discrete_PID_EB_FF subsystem
ffFlagPath = [model_name '/discrete_PID_EB_FF/Constant'];


modes = [2, 1];   % 2 = FF off (pure PID_EB), 1 = FF on
modeNames = {'NoFF','WithFF'};

% prepare storage
ResultsFF = table('Size',[2 3], ...
    'VariableTypes',{'double','double','double'}, ...
    'RowNames',modeNames, ...
    'VariableNames',{'MaxAbsError_deg','RMSError_deg','P2PError_deg'});

errData = struct();

for k = 1:2
  
  set_param(ffFlagPath, 'Value', num2str(modes(k)));
  set_param(model_name, 'SimulationCommand', 'update');
  set_param(model_name, 'SimulationCommand', 'Stop');  % force refresh
   
  pause(0.1);  % allow Simulink to re-evaluate everything
  
  simOut = sim(model_name,'StopTime','5','ReturnWorkspaceOutputs','on');
  
  % extract error signal and time
  e = simOut.get('pos_error_FF').Data;        % degrees
  t = simOut.get('pos_error_FF').Time;
  
  % compute metrics
  maxAbs = max(abs(e));
  rmsErr = sqrt(mean(e.^2));
  p2p    = max(e) - min(e);
  
  % store
  ResultsFF{modeNames{k},:} = [maxAbs, rmsErr, p2p];
  errData.(modeNames{k}) = struct('t',t,'e',e);
end

% display table
disp('=== Error metrics (accurate model) ===');
disp(ResultsFF);

% plot error traces
figure;
hold on;
p1 = plot(errData.NoFF.t,    errData.NoFF.e,    'r--','LineWidth',1.5);
p2 = plot(errData.WithFF.t, errData.WithFF.e, 'b-','LineWidth',1.5);
xlabel('Time (s)');
ylabel('Position error (deg)');
title('Tracking error: No FF vs With FF (accurate model)');
legend([p1 p2],{'Error without FF','Error with FF'},'Location','northeast');
grid on;

% Save table to CSV
csv_filename = 'ff_comparison.csv';
writetable(ResultsFF, csv_filename, 'WriteRowNames', true);
fprintf('FF comparison metrics saved to: %s\n', csv_filename);

% Save error plot to PNG
fig_filename = 'ff_error_plot.png';
saveas(gcf, fig_filename);
fprintf('FF error plot saved to: %s\n', fig_filename);

close_system("Model_Blackbox_system_discrete_PIDs.slx",0);

