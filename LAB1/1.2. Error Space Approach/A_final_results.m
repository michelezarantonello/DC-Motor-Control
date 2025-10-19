%% Automation Script: Error-Space Controller Validation (Black-Box DC Motor)
% This script automates testing of an error-space controller for a black-box DC motor model in Simulink.
% It performs:
%   1. Sinusoidal period variation tests (controller redesigned each time)
%   2. Amplitude sweep at T_r = 0.5 s (controller designed for 0.5 s only)
%   3. Cross-test at T_r = 0.1 s with controller designed at T_r = 0.5 s
%   4. Step reference tracking test
%
% The model contains:
%  - A Transfer Fcn block named 'Feedforward' for the feed‑forward numerator K_z
%  - A Gain block named 'K_chi' for the state‑feedback K_chi
%  - A Multiport Switch constant named 'Constant' selecting reference type
%  - A Sine Wave block named 'Sinusoidal Reference' (in degrees)
%  - A Step block named 'Step Reference' (in degrees)

%% Setup

run("Plant_EstimTrialUpdateLAB17_04_25.m");

open("Model_RobErrorSpaceSSM.slx");
model_name = 'Model_RobErrorSpaceSSM';

set_param(model_name, 'SimulationCommand', 'update');
pause(0.1);


% Paths to blocks (adjust these names if your blocks differ)
inport_ref_selector   = [model_name '/Constant'];            % Multiport switch control
inport_sin_block      = [model_name '/Sinusoidal Reference'];% Sine Wave block (deg)
inport_step_block     = [model_name '/Step Reference'];      % Step block (deg)
controller_gain_block = [model_name '/K_chi'];               % State-feedback Gain block
ff_block              = [model_name '/H(s)'];                % Transfer Fcn block for feedforward

%% Load motor parameters
run('Plant_EstimTrialUpdateLAB17_04_25.m'); % defines Tm, km, N, etc.

%% Initialize Result Tables
table_columns_sine = {'T_r','rms_err','max_err','ss_rms','settling_time'};
table_columns_amp  = {'A_r','rms_err','max_err','ss_rms','settling_time'};
results_sine      = cell2table(cell(0,numel(table_columns_sine)), 'VariableNames',table_columns_sine);
results_amp_sweep = cell2table(cell(0,numel(table_columns_amp)),  'VariableNames',table_columns_amp);

%% Test 1: Sine Sweep with controller redesign for each T_r

T_r_set   = [0.15, 0.25, 0.5, 1];
A_r_sine  = 30;   % amplitude in degrees
ss_window = 2;    % seconds for steady-state RMS window

set_param(inport_ref_selector,'Value','1');
set_param(inport_sin_block,'Amplitude', num2str(A_r_sine));
set_param(inport_sin_block,'Frequency', num2str(2*pi*(1/T_r_set(3))));

for T_r = T_r_set

    % Design controller + feedforward for this period
    [K_chi, K_z, omega_input] = Controller_ErrorStateApproach(T_r, Tm, km, N);

    % Push gains into base workspace for Simulink
    assignin('base','K_chi',K_chi);
    assignin('base','K_z',  K_z);
    assignin('base', 'omega_input', omega_input);

    %denominator coefficients:
    d = [1 0 omega_input^2 0];
    
    % Configure Controller Blocks:

    % K_chi ~ Original SSM Feedback:
    set_param(controller_gain_block,'Gain',      mat2str(K_chi));
    
    % K_z(1:3) ~ Error-Subspace Feedforward
    set_param(ff_block,             'Numerator', mat2str(fliplr(K_z(1:3))));
    
    % d ~ LCM(reference & disturbance) perfect tracking/rejection
    set_param(ff_block,             'Denominator',mat2str(d));

    % Configure sinusoidal reference
    set_param(inport_ref_selector,'Value','1');
    set_param(inport_sin_block,'Amplitude', num2str(A_r_sine));
    set_param(inport_sin_block,'Frequency', num2str(2*pi/T_r));

    % Run simulation
    simOut = sim(model_name,'ReturnWorkspaceOutputs','on');
    e = simOut.get('error_signal').Data;  % error in degrees
    t = simOut.get('error_signal').Time;

    % Compute metrics
    rms_err = rms(e);
    max_err = max(abs(e));

    idx_ss  = t >= t(end)-ss_window;
    

    if isempty(idx_ss)
        ss_rms = rms_err;
    else
        ss_rms = rms(e(idx_ss));
    end


    settling_time = NaN;

    % Store
    results_sine = [results_sine; table(T_r, rms_err, max_err, ss_rms, settling_time)];
end

%% Test 2: Amplitude Sweep at T_r = 0.5 s (fixed controller)

T_r_fixed   = 0.5;

A_r_set     = [30, 60, 90];
ss_window   = 2;

% Reuse controller/feedforward from T_r_fixed

[K_chi_fixed, K_z_fixed, omega_fixed] = Controller_ErrorStateApproach(T_r_fixed, Tm, km, N);

assignin('base','K_chi',K_chi_fixed);

assignin('base','K_z',  K_z_fixed);

assignin('base', 'omega_fixed', omega_fixed);

d = [1 0 omega_fixed^2 0];

set_param(controller_gain_block,'Gain',      mat2str(K_chi_fixed));

set_param(ff_block,             'Numerator', mat2str(fliplr(K_z(1:3))));

set_param(ff_block,             'Denominator',mat2str(d));

for A_r = A_r_set
    set_param(inport_ref_selector,'Value','1');
    set_param(inport_sin_block,'Amplitude', num2str(A_r));
    set_param(inport_sin_block,'Frequency', num2str(omega_fixed));

    

    simOut = sim(model_name,'ReturnWorkspaceOutputs','on');
    e = simOut.get('error_signal').Data;
    t = simOut.get('error_signal').Time;

    rms_err = rms(e);
    max_err = max(abs(e));
    idx_ss  = t >= t(end)-ss_window;

        if isempty(idx_ss)
        ss_rms = rms_err;
    else
        ss_rms = rms(e(idx_ss));
    end
    settling_time = NaN;

    results_amp_sweep = [results_amp_sweep; table(A_r, rms_err, max_err, ss_rms, settling_time)];
end

%% Test 3: Cross-Test at T_r = 0.1 s with fixed controller

T_r_cross   = 0.1;
omega_cross = 2*pi/T_r_cross;
A_r_cross   = 40;
ss_window   = 2;

assignin('base','K_chi',K_chi_fixed);
assignin('base','K_z',  K_z_fixed);

assignin('base', 'omega_fixed', omega_fixed);

d = [1 0 omega_fixed^2 0];

set_param(controller_gain_block,'Gain',      mat2str(K_chi_fixed));
set_param(ff_block,             'Numerator', mat2str(fliplr(K_z(1:3))));

set_param(ff_block,             'Denominator',mat2str(d));

set_param(inport_ref_selector,'Value','1');
set_param(inport_sin_block,'Amplitude', num2str(A_r_cross));
set_param(inport_sin_block,'Frequency', num2str(omega_cross));

simOut = sim(model_name,'ReturnWorkspaceOutputs','on');
 e = simOut.get('error_signal').Data;
 t = simOut.get('error_signal').Time;

cross_rms = rms(e);
cross_max = max(abs(e));
idx_ss    = t >= t(end)-ss_window;

if isempty(idx_ss)
    cross_ss_rms = cross_rms;
else
    cross_ss_rms = rms(e(idx_ss));
end

cross_settling_time = NaN;
results_cross = table(T_r_cross, A_r_cross, cross_rms, cross_max, cross_ss_rms, cross_settling_time, ...
    'VariableNames',{'T_r','A_r','rms_err','max_err','ss_rms','settling_time'});

%% Test 4: Step Reference Test (40 deg) with fixed controller
A_step = 40;

assignin('base','K_chi',K_chi_fixed);
assignin('base','K_z',  K_z_fixed);

assignin('base', 'omega_fixed', omega_fixed);

d = [1 0 omega_fixed^2 0];

set_param(controller_gain_block,'Gain',      mat2str(K_chi_fixed));
set_param(ff_block,             'Numerator', mat2str(fliplr(K_z(1:3))));

set_param(ff_block,             'Denominator',mat2str(d));


set_param(inport_ref_selector,'Value','2');
set_param(inport_step_block,'After', num2str(A_step));

simOut = sim(model_name,'ReturnWorkspaceOutputs','on');
y = simOut.get('position_meas').Data;
t = simOut.get('position_meas').Time;

stepinfo_result = stepinfo(y, t, 'RiseTimeLimits',[0.1 0.9],'SettlingTimeThreshold',0.05);
final_error = abs(y(end) - A_step);
step_settling_time = stepinfo_result.SettlingTime;
step_rise_time = stepinfo_result.RiseTime;
step_overshoot = stepinfo_result.Overshoot;
results_step = table(A_step, step_rise_time, step_settling_time, step_overshoot, final_error, ...
    'VariableNames',{'A_step','RiseTime_s','SettlingTime_s','Overshoot_pct','FinalError'});

%% Display Results
disp('=== Test 1: Sine Sweep Results ==='); disp(results_sine);
disp('=== Test 2: Amplitude Sweep Results ==='); disp(results_amp_sweep);
disp('=== Test 3: Cross-Test Results ==='); disp(results_cross);
disp('=== Test 4: Step Reference Test Results ==='); disp(results_step);

%% Save Results\ nw1 = 'results_test1_sine_sweep.csv'; writetable(results_sine, nw1);
writetable(results_amp_sweep, 'results_test2_amp_sweep.csv');
writetable(results_cross,       'results_test3_cross.csv');
writetable(results_step,        'results_test4_step.csv');

close_system("Model_RobErrorSpaceSSM.slx",0);
