 % J_eq Estimation with automatic phase detection via zero crossings

load("TorqueForJeq.mat"); 
% variable name: TorqueNew
load("AccelerationForJeq.mat"); 
% variable name: RealAcceleration
load("SpeedForJeq.mat")
% variable name: RealSpeedNew

T_m = TorqueNew.signals.values; 
speed = RealSpeedNew.signals.values;


T_f_prime = B_eq_avg * speed + tau_sf_avg * gear_ratio * sign(speed);

T = T_m - T_f_prime;


A = RealAcceleration.signals.values; 
N = length(T); 
t = (0:N-1)*0.001;

% (Optional) Smooth the acceleration signal to reduce small-noise-induced zero crossings 
A_smooth = movmean(A, 10);

% Find indices where the sign of A_smooth changes % (i.e. from positive to negative or vice versa) 
crossings = find(diff(sign(A_smooth))~=0);

% To include the first and last sample as boundaries: 
if isempty(crossings) || crossings(1) ~= 1 
    crossings = [1; crossings]; 
end 

if crossings(end) ~= N 
    crossings = [crossings; N]; 
end

% Number of detected phases is one less than the number of crossing points 
numPhases = length(crossings) - 1;

% Initialize arrays to hold the mean values during each phase 
T_phase_pos = []; 

% torque values for phases with positive acceleration 
A_phase_pos = []; 

% acceleration values for phases with positive acceleration 
T_phase_neg = []; 

% torque for negative acceleration phases 
A_phase_neg = []; 

% acceleration for negative phases

% For visualization, we will reconstruct an "average" signal 
T_visualized = nan(1,N); 
A_visualized = nan(1,N);

% Loop through each detected phase interval 
for p = 1:numPhases 
    phase_start = crossings(p);
    phase_end = crossings(p+1);
    phase_length = phase_end - phase_start + 1;

% Discard first and last 10% of the interval to avoid transients
discard = round(0.1 * phase_length);
idx_phase = (phase_start+discard) : (phase_end-discard);

if isempty(idx_phase)
    continue;  % In case the interval is too short
end

% Compute mean torque and acceleration over this stable part of the phase
T_mean = mean( T(idx_phase) );
A_mean = mean( A(idx_phase) );

% Save for visualization (we fill the entire phase with the mean)
T_visualized(idx_phase) = T_mean;
A_visualized(idx_phase) = A_mean;

% Classify the phase by the sign of the acceleration (mean value)
if A_mean > 0
    T_phase_pos(end+1) = T_mean;
    A_phase_pos(end+1) = A_mean;
elseif A_mean < 0
    T_phase_neg(end+1) = T_mean;
    A_phase_neg(end+1) = A_mean;
end
% (If A_mean equals 0 exactly, you may choose to ignore or handle it separately.)  
end

% Now, estimate J_eq using the average of the positive and negative phases. 


ti_pos = mean(T_phase_pos); 
ti_neg = mean(T_phase_neg); 
acc_pos = mean(A_phase_pos); 
acc_neg = mean(A_phase_neg);

Jeq = (ti_pos - ti_neg) / (acc_pos - acc_neg)

fprintf('Estimated J_eq = %f\n', Jeq);

% Visualization of raw data with averaged phases overlaid
 
figure; 
subplot(2,1,1); 
plot(t, T, 'b'); 
hold on; 
plot(t, T_visualized, 'k', 'LineWidth', 2); 
title('Raw Torque Data with Phase Averages'); 
xlabel('Time [s]'); 
ylabel('Torque');

subplot(2,1,2); 
plot(t, A, 'r'); 
hold on; 
plot(t, A_visualized, 'k', 'LineWidth', 2); 
title('Raw Acceleration Data with Phase Averages'); 
xlabel('Time [s]'); 
ylabel('Acceleration');