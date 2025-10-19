%% Automation Script: Extended Estimator Controller Validation (Black-Box DC Motor)
% This script automates testing of an extended estimator controller for a black-box DC motor model in Simulink.
% It performs tests with controller designed at T_r = 0.5 s for:
%   
%   2. Amplitude sweep at A_r_set = [30, 60, 90] deg
%   3. Cross-test at sinusoidal reference of period T_r = 0.1 s 
%   4. Step reference with A = 40 deg
%
% The model contains:
%  - A Transfer Fcn block named 'Feedforward' for the feed‑forward numerator K_z
%  - A Gain block named 'K_chi' for the state‑feedback K_chi
%  - A Multiport Switch constant named 'Constant' selecting reference type
%  - A Sine Wave block named 'Sinusoidal Reference' (in degrees)
%  - A Step block named 'Step Reference' (in degrees)


%% Setup
model_name = 'Model_extended_estimator_bb';

run("Plant_EstimTrialUpdateLAB17_04_25.m"); % defines Tm, km, N, etc.

% Paths to blocks (adjust these names if your blocks differ)
inport_ref_selector   = [model_name '/Switch'];              % Multiport switch control
inport_sin_block      = [model_name '/Sine Wave'];           % Sine Wave block (deg)
inport_step_block     = [model_name '/Step'];                % Step block (deg)

%% Initialize Result Tables
table_columns_sine = {'T_r','rms_err','max_err','ss_rms','settling_time'};
table_columns_amp  = {'A_r','rms_err','max_err','ss_rms','settling_time'};
results_sine      = cell2table(cell(0,numel(table_columns_sine)), 'VariableNames',table_columns_sine);
results_amp_sweep = cell2table(cell(0,numel(table_columns_amp)),  'VariableNames',table_columns_amp);

%% Test 1: Amplitude Sweep at T_r = 0.5 s (fixed controller)

T_r_fixed   = 0.5;
omega_fixed = 2*pi/T_r_fixed;

A_r_set     = [30, 60, 90];   % amplitudes in degrees
ss_window   = 2;              % seconds to consider for steady state


% Reuse controller/feedforward from T_r_fixed
[Ae, Be, Ce, Le, p, p_obs, C_rho, K_stateFB_extended] = Controller_StateSpace_Lab1_25_04_25(T_r_fixed, Tm, km, N);

open("Model_extended_estimator_bb.slx");
set_param(model_name, 'SimulationCommand', 'update');
pause(0.1);

% Get simulation step size from model if not predefined
T = str2double(get_param(model_name, 'FixedStep'));  

% Preallocate results table
results_amp_sweep = table('Size', [length(A_r_set), 5], ...
                          'VariableTypes', {'double','double','double','double','double'}, ...
                          'VariableNames', {'Amplitude', 'RMS_Error', 'Max_Abs_Error', 'SteadyState_RMS', 'SteadyState_PeakToPeak'});

for i = 1:length(A_r_set)
    A_r = A_r_set(i);

    % Update sinusoidal reference parameters
    set_param(inport_ref_selector, 'Value', '1');  % 1 = Sinusoidal
    set_param(inport_sin_block, 'Amplitude', num2str(A_r));
    set_param(inport_sin_block, 'Frequency', num2str(omega_fixed));

    % Run simulation
    simOut = sim(model_name, 'ReturnWorkspaceOutputs','on');

    % Extract signals
    bb_sig    = simOut.get('pos_meas');
    bb_refSig = simOut.get('pos_ref');
    t_bb      = bb_sig.Time;
    y_bb      = bb_sig.Data;
    ref_bb    = interp1(bb_refSig.Time, bb_refSig.Data, t_bb);
    e_bb      = ref_bb - y_bb;

    % Global metrics
    rms_err   = sqrt(trapz(t_bb, e_bb.^2) / (t_bb(end)-t_bb(1)));
    max_err   = max(abs(e_bb));

    % Steady-state metrics (last ss_window seconds)
    t_ss_start = t_bb(end) - ss_window;
    ss_mask    = t_bb >= t_ss_start;
    e_ss       = e_bb(ss_mask);
    t_ss       = t_bb(ss_mask);
    
    ss_rms     = sqrt(trapz(t_ss, e_ss.^2) / (t_ss(end)-t_ss(1)));
    ss_pkpk    = max(e_ss) - min(e_ss);

    % Store results
    results_amp_sweep{i, :} = [A_r, rms_err, max_err, ss_rms, ss_pkpk];
end

% Display table
disp(results_amp_sweep)

%% Test 3: Cross-Test at T_r = 0.1 s with 0.5s designed controller

T_r_cross   = 0.1;
omega_cross = 2*pi/T_r_cross;
A_r_cross   = 40;
ss_window   = 2;

set_param(inport_ref_selector, 'Value', '1');  % Sinusoidal mode
set_param(inport_sin_block, 'Amplitude', num2str(A_r_cross));
set_param(inport_sin_block, 'Frequency', num2str(omega_cross));

% Run simulation
simOut = sim(model_name, 'ReturnWorkspaceOutputs','on');

% Extract signals
bb_sig    = simOut.get('pos_meas');
bb_refSig = simOut.get('pos_ref');
t_bb      = bb_sig.Time;
y_bb      = bb_sig.Data;
ref_bb    = interp1(bb_refSig.Time, bb_refSig.Data, t_bb);
e_bb      = ref_bb - y_bb;

% Global metrics
cross_rms     = sqrt(trapz(t_bb, e_bb.^2) / (t_bb(end)-t_bb(1)));
cross_max     = max(abs(e_bb));

% Steady-state metrics (last ss_window seconds)
t_ss_start = t_bb(end) - ss_window;
ss_mask    = t_bb >= t_ss_start;
e_ss       = e_bb(ss_mask);
t_ss       = t_bb(ss_mask);

cross_ss_rms  = sqrt(trapz(t_ss, e_ss.^2) / (t_ss(end)-t_ss(1)));
cross_pp      = max(e_ss) - min(e_ss);  % If you want to include this too

% Settling time not meaningful for sinusoids; set NaN
cross_settling_time = NaN;

% Store results in table
results_cross = table(T_r_cross, A_r_cross, cross_rms, cross_max, cross_ss_rms, cross_settling_time, ...
    'VariableNames', {'T_r','A_r','rms_err','max_err','ss_rms','settling_time'});

% Display
disp(results_cross);


%% Test 4: Step Reference Test (40 deg) with 0.5s designed controller
A_step = 40;

set_param(inport_ref_selector,'Value','2');
set_param(inport_step_block,'After', num2str(A_step));

simOut = sim(model_name,'ReturnWorkspaceOutputs','on');
y = simOut.get('pos_meas').Data;
t = simOut.get('pos_meas').Time;

stepinfo_result = stepinfo(y, t, 'RiseTimeLimits',[0.1 0.9],'SettlingTimeThreshold',0.05);
final_error = abs(y(end) - A_step);
step_settling_time = stepinfo_result.SettlingTime;
step_rise_time = stepinfo_result.RiseTime;
step_overshoot = stepinfo_result.Overshoot;

results_step = table(A_step, step_rise_time, step_settling_time, step_overshoot, final_error, ...
    'VariableNames',{'A_step','RiseTime_s','SettlingTime_s','Overshoot_pct','FinalError'});

%% Display Results
disp('=== Test 2: Amplitude Sweep Results ==='); disp(results_amp_sweep);
disp('=== Test 3: Cross-Test Results ===');     disp(results_cross);
disp('=== Test 4: Step Reference Test Results ==='); disp(results_step);

%% Save Results
writetable(results_amp_sweep,  'results_test2_amp_sweep.csv');
writetable(results_cross,      'results_test3_cross.csv');
writetable(results_step,       'results_test4_step.csv');

close_system("Model_extended_estimator_bb.slx",0);
