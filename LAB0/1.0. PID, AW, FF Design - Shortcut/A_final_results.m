%% DC Motor PID Design, AW and Feedforward Mechanisms

%% === Performance Specifications and Results ===
% 
% For Step Reference Tracking (PID, PID + AW):
%
%  - Rise Time (s): 
%      Time taken for the output to rise from 10% to 90% of the final value.
%      Computed using 'RiseTimeLimits' = [0.1 0.9].
%
%  - Settling Time (s): 
%      Time taken for the output to remain within 5% of the final value without leaving again.
%      Computed using 'SettlingTimeThreshold' = 0.05.
%
%  - Overshoot (%):
%      Maximum percentage by which the response exceeds the reference value during transient.
%
% These metrics are extracted from step responses using the stepinfo() function.
%
%
% For Sinusoidal Reference Tracking (PID + AW + FF):
%
%  - Max Peak Error (deg):
%      Maximum absolute value of the tracking error measured at the key peak and trough instants 
%      of the reference sine wave (times: t = 1.5 + k*(T/2) seconds).
%      Indicates worst-case tracking mismatch when the system is under maximum or minimum excursion.
%
%  - RMS Error (deg):
%      Root Mean Square of the tracking error over the observation window.
%      Provides a measure of the average energy of the tracking error.
%      Computed as:
%         e_rms = sqrt( (1/(t_end - t_start)) * integral(e(t)^2 dt) )
%
%  - Peak-to-Peak Error (deg):
%      Difference between the maximum and minimum tracking error over the entire test duration:
%         e_pp = max(e(t)) - min(e(t))
%      Captures the full swing of tracking error fluctuations, including intermediate times.
%
% 
% General Notes:
%
%  - Tracking Error e(t) is defined as:
%        e(t) = Reference(t) - Measured(t)
%
%  - For sinusoidal tracking, interpolation is used to synchronize reference and measured signals 
%    onto a common time vector before error computation.
%
%  - The goal of Feedforward Compensation is to reduce Max Peak Error, RMS Error, and Peak-to-Peak Error
%    by anticipating system dynamics (Inertia, Friction and Back EMF Compensation).

%% Load motor parameters
run('Plant_EstimTrialUpdateLAB17_04_25.m'); % defines Tm, km, N, etc.

%% Setup References to Test
A_set = [10, 30, 50, 90, 180, 360];  % degrees
controller_labels = ["PID", "PID + AW"];


% Result Tables
results_acc = table();
results_bb  = table();

model_name = 'Model_test_system_simul';

% Open the Simulink Model, Update and Pause to ensure proper loading
open("Model_test_system_simul.slx");

set_param(model_name, 'SimulationCommand', 'update');
pause(0.1);

for ctrl_mode = 1:2  % 1 = PID, 2 = PID + AW

    set_param([model_name '/Model to Test'], 'Value',num2str(ctrl_mode));

    for a = A_set
        % Set reference amplitude (degrees) in the Step block
        set_param([model_name '/Position reference [deg]'], 'After', num2str(a));

        % Run simulation
        simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');

        % Extract measured responses
        pos_acc = simOut.get('pos_meas_acc');
        pos_bb  = simOut.get('pos_meas_bb');

        t_acc = pos_acc.time;
        y_acc = pos_acc.signals.values(:, 1);
        t_bb  =  pos_bb.time;
        y_bb  =  pos_bb.signals.values(:, 1);

        % Compute metrics
        info_acc = stepinfo(y_acc, t_acc, 'RiseTimeLimits', [0.1 0.9], 'SettlingTimeThreshold', 0.05);
        info_bb  = stepinfo(y_bb,  t_bb,  'RiseTimeLimits', [0.1 0.9], 'SettlingTimeThreshold', 0.05);

        % Append to tables
        results_acc = [results_acc; 
            table(controller_labels(ctrl_mode), a, info_acc.RiseTime, info_acc.SettlingTime, info_acc.Overshoot, ...
                  'VariableNames', {'Controller', 'RefAmplitude_deg', 'RiseTime', 'SettlingTime', 'Overshoot'})];

        results_bb = [results_bb; 
            table(controller_labels(ctrl_mode), a, info_bb.RiseTime, info_bb.SettlingTime, info_bb.Overshoot, ...
                  'VariableNames', {'Controller', 'RefAmplitude_deg', 'RiseTime', 'SettlingTime', 'Overshoot'})];
    end
end

% Save results
writetable(results_acc, 'Results_Accurate_Model.csv');
writetable(results_bb, 'Results_BlackBox_Model.csv');

disp("=== Test Complete ===");
disp("Accurate Model:");
disp(results_acc);
disp("Black Box Model:");
disp(results_bb);


%% Plot performance trends: Accurate Model
figure('Name', 'Performance - Accurate Model');

% Highlight index
highlight_deg = 360;

% Filter values
g1 = results_acc.RefAmplitude_deg(results_acc.Controller == "PID");

% Rise Time
subplot(3,1,1);
hold on;
pid_rt     = results_acc.RiseTime(results_acc.Controller == "PID");
pid_aw_rt  = results_acc.RiseTime(results_acc.Controller == "PID + AW");
plot(g1, pid_rt, 'b-o', 'DisplayName', 'PID');
plot(g1, pid_aw_rt, 'r-s', 'DisplayName', 'PID + AW');
highlight_idx = find(g1 == highlight_deg);
plot(g1(highlight_idx), pid_rt(highlight_idx), 'gs', 'MarkerSize', 8, 'DisplayName', '360° PID');
plot(g1(highlight_idx), pid_aw_rt(highlight_idx), 'gs', 'MarkerSize', 8, 'DisplayName', '360° PID+AW');
ylabel('Rise Time (s)');
legend('PID','PID + AW','Location','northwest'); grid on; title('Accurate Model - Rise Time');

% Settling Time
subplot(3,1,2);
hold on;
pid_st     = results_acc.SettlingTime(results_acc.Controller == "PID");
pid_aw_st  = results_acc.SettlingTime(results_acc.Controller == "PID + AW");
plot(g1, pid_st, 'b-o');
plot(g1, pid_aw_st, 'r-s');
plot(g1(highlight_idx), pid_st(highlight_idx), 'gs', 'MarkerSize', 8);
plot(g1(highlight_idx), pid_aw_st(highlight_idx), 'gs', 'MarkerSize', 8);
ylabel('Settling Time (s)');
legend('PID','PID + AW','Location','northwest'); grid on; title('Accurate Model - Settling Time');

% Overshoot
subplot(3,1,3);
hold on;
pid_os     = results_acc.Overshoot(results_acc.Controller == "PID");
pid_aw_os  = results_acc.Overshoot(results_acc.Controller == "PID + AW");
plot(g1, pid_os, 'b-o');
plot(g1, pid_aw_os, 'r-s');
plot(g1(highlight_idx), pid_os(highlight_idx), 'gs', 'MarkerSize', 8);
plot(g1(highlight_idx), pid_aw_os(highlight_idx), 'gs', 'MarkerSize', 8);
ylabel('Overshoot (%)');
xlabel('Reference Amplitude (deg)');
legend('PID','PID + AW','Location','northeast'); grid on; title('Accurate Model - Overshoot');

saveas(gcf, fullfile(pwd, 'performance_trends_accurate.png'));

%% Plot performance trends: Black Box Model
figure('Name', 'Performance - Black Box Model');

g2 = results_bb.RefAmplitude_deg(results_bb.Controller == "PID");

% Rise Time
subplot(3,1,1);
hold on;
pid_rt     = results_bb.RiseTime(results_bb.Controller == "PID");
pid_aw_rt  = results_bb.RiseTime(results_bb.Controller == "PID + AW");
plot(g2, pid_rt, 'b-o', 'DisplayName', 'PID');
plot(g2, pid_aw_rt, 'r-s', 'DisplayName', 'PID + AW');
highlight_idx = find(g2 == highlight_deg);
plot(g2(highlight_idx), pid_rt(highlight_idx), 'gs', 'MarkerSize', 8, 'DisplayName', '360° PID');
plot(g2(highlight_idx), pid_aw_rt(highlight_idx), 'gs', 'MarkerSize', 8, 'DisplayName', '360° PID+AW');
ylabel('Rise Time (s)');
legend('PID','PID + AW','Location','northwest'); grid on; title('Black Box Model - Rise Time');

% Settling Time
subplot(3,1,2);
hold on;
pid_st     = results_bb.SettlingTime(results_bb.Controller == "PID");
pid_aw_st  = results_bb.SettlingTime(results_bb.Controller == "PID + AW");
plot(g2, pid_st, 'b-o');
plot(g2, pid_aw_st, 'r-s');
plot(g2(highlight_idx), pid_st(highlight_idx), 'gs', 'MarkerSize', 8);
plot(g2(highlight_idx), pid_aw_st(highlight_idx), 'gs', 'MarkerSize', 8);
ylabel('Settling Time (s)');
legend('PID','PID + AW','Location','northwest'); grid on; title('Black Box Model - Settling Time');

% Overshoot
subplot(3,1,3);
hold on;
pid_os     = results_bb.Overshoot(results_bb.Controller == "PID");
pid_aw_os  = results_bb.Overshoot(results_bb.Controller == "PID + AW");
plot(g2, pid_os, 'b-o');
plot(g2, pid_aw_os, 'r-s');
plot(g2(highlight_idx), pid_os(highlight_idx), 'gs', 'MarkerSize', 8);
plot(g2(highlight_idx), pid_aw_os(highlight_idx), 'gs', 'MarkerSize', 8);
ylabel('Overshoot (%)');
xlabel('Reference Amplitude (deg)');
legend('PID','PID + AW','Location','northeast'); grid on; title('Black Box Model - Overshoot');

saveas(gcf, fullfile(pwd, 'performance_trends_blackbox.png'));

%% ==== Feedforward OFF vs ON (FF) with mixed metrics ==== 

% Results show that the no-FF implementation lags behind the reference,
% creating a sinusoidal error signal. The FF terms compensate this lag,
% giving high-precision tracking performance.

model_name       = 'Model_test_system_simul';

%Set the controller into 'PID+AW+FF' mode:

main_switch_path = [model_name '/Model to Test'];
set_param(main_switch_path, 'Value', num2str(3));

test_switch_path = [model_name '/Subsystem Reference1/Constant'];  % turn on/off FF compensation

modes    = [2, 1];               % 2 = No FF (step), 1 = With FF 

results_ff = table([], [], [], [], [], [], ...
  'VariableNames', ["Acc_MaxPeak_deg","Acc_RMS_deg","Acc_P2P_deg", ...
                    "BB_MaxPeak_deg","BB_RMS_deg","BB_P2P_deg"]);

results_no_ff = table([], [], [], [], [], [], ...
  'VariableNames', ["Acc_MaxPeak_deg","Acc_RMS_deg","Acc_P2P_deg", ...
                    "BB_MaxPeak_deg","BB_RMS_deg","BB_P2P_deg"]);

% Error storage
acc_errors = cell(2,1);  acc_time = cell(2,1);
bb_errors  = cell(2,1);  bb_time  = cell(2,1);

for k = 1:2

  % select mode
  set_param(test_switch_path, 'Value', num2str(modes(k)));
  simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');

  acc_sig    = simOut.get('pos_meas_acc');
  acc_refSig = simOut.get('pos_PID_FF_acc');
  t_acc      = acc_sig.time;
  y_acc      = acc_sig.signals.values;
  ref_acc    = interp1(acc_refSig.time, acc_refSig.signals.values, t_acc);
  e_acc      = ref_acc - y_acc;

  % No FF — black box
  bb_sig    = simOut.get('pos_meas_bb');
  bb_refSig = simOut.get('pos_PID_FF_bb');
  t_bb      = bb_sig.time;
  y_bb      = bb_sig.signals.values;
  ref_bb    = interp1(bb_refSig.time, bb_refSig.signals.values, t_bb);
  e_bb      = ref_bb - y_bb;

  acc_time{k} = t_acc;  acc_errors{k} = e_acc;
  bb_time{k}  = t_bb;   bb_errors{k}  = e_bb;

  % Custom error metrics
  T = 3;
  pkA = 1.5 : T/2 : t_acc(end);
  pkB = 1.5 : T/2 : t_bb(end);
  epA = interp1(t_acc, e_acc, pkA);
  epB = interp1(t_bb,  e_bb,  pkB);

  if modes(k) == 2
    % No FF — accurate model

    results_no_ff = [results_no_ff; {
      max(abs(epA)), sqrt(trapz(t_acc, e_acc.^2)/(t_acc(end)-t_acc(1))), max(e_acc) - min(e_acc), ...
      max(abs(epB)), sqrt(trapz(t_bb, e_bb.^2)/(t_bb(end)-t_bb(1))), max(e_bb) - min(e_bb)}];
  else

    results_ff = [results_ff; {
      max(abs(epA)), sqrt(trapz(t_acc, e_acc.^2)/(t_acc(end)-t_acc(1))), max(e_acc) - min(e_acc), ...
      max(abs(epB)), sqrt(trapz(t_bb, e_bb.^2)/(t_bb(end)-t_bb(1))), max(e_bb) - min(e_bb)}];
  end
end

% display & save
disp("=== FF Tracking Metrics ===");
disp(results_ff);
writetable(results_ff,'ff_performance.csv');

% display & save
disp("=== No FF Tracking Metrics ===");
disp(results_no_ff);
writetable(results_no_ff,'step_performance.csv');

%–– plot the two error traces –– 
figure('Name','Tracking Error: No FF vs FF');

subplot(2,1,1); hold on;
p1 = plot(acc_time{1}, acc_errors{1}, 'r--', 'LineWidth', 1.5);
p2 = plot(acc_time{2}, acc_errors{2}, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (deg)');
title('Accurate Model Tracking Error');
legend([p1 p2], {'Error without FF', 'Error with FF'}, 'Location', 'northeast');
grid on; xlim([0 max(acc_time{2})]);

subplot(2,1,2); hold on;
p3 = plot(bb_time{1}, bb_errors{1}, 'r--', 'LineWidth', 1.5);
p4 = plot(bb_time{2}, bb_errors{2}, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (deg)');
title('Black-Box Model Tracking Error');
legend([p3 p4], {'Error without FF', 'Error with FF'}, 'Location', 'northeast');
grid on; xlim([0 max(bb_time{2})]);

saveas(gcf, fullfile(pwd, 'no_ff_vs_ff_tracking_error.png'));

close_system("Model_test_system_simul.slx",0); %0 to discard changes done by testing


