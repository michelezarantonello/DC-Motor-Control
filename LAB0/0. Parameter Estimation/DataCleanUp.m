function rolling_avg = DataCleaner(T, interval_sec, remove_sec, w, label)

    N = length(T);
    fs = 1000;  % sampling frequency

    samples_per_interval = round(interval_sec * fs);
    samples_to_remove = round(remove_sec * fs);
    
    % 1. Removal of transient spikes:
    new_signal = [];
    i = 1;
    
    while i <= N
        if mod(i-1, samples_per_interval) == 0 && i ~= 1
            i = i + samples_to_remove;
            if i > N
                break;
            end
        end
        new_signal(end+1) = T(i);
        i = i + 1;
    end

    % 2. Rolling average
    n = length(new_signal);
    rolling_avg = zeros(1, n - w + 1);
    
    for i = 1:(n - w + 1)
        rolling_avg(i) = mean(new_signal(i:i+w-1));
    end

    % 3. Plotting
    t = (0:N-1) * 0.001;
    ttt = (0:length(new_signal)-1) * 0.001;
    tt = (0:length(rolling_avg)-1) * 0.001;

    figure('Name', label, 'NumberTitle', 'off');
    
    subplot(2,2,1); plot(t, T); title('1. Raw Signal Data');
    subplot(2,2,2); plot(ttt, new_signal, 'r'); title('2. Spliced Signal');
    subplot(2,2,3); plot(tt, rolling_avg, 'k'); title('3. Averaged Signal');

    % Save figure with sanitized label
    clean_label = lower(strrep(label, ' ', '_'));
    saveas(gcf, fullfile(pwd, [clean_label, '.png']));

end

load("PositiveMotorTorqueStair.mat"); 
T_pos = Torque.signals.values;
avg_pos_T = DataCleaner(T_pos, 5, 0.5, 300, 'Positive Torque Profile');
save('Averaged_Positive_Torque_Profile.mat', 'avg_pos_T');

load("NegativeMotorTorqueStair.mat"); 
T_neg = Torque.signals.values;
avg_neg_T = DataCleaner(T_neg, 5, 0.5, 300, 'Negative Torque Profile');
save('Averaged_Negative_Torque_Profile.mat', 'avg_neg_T');

load("PositiveMotorSpeedStair.mat"); 
S_pos = Position.signals.values;
avg_pos_S = DataCleaner(S_pos, 5, 0.5, 300, 'Positive Speed Profile');
save('Averaged_Positive_Speed_Profile.mat', 'avg_pos_S');

load("NegativeMotorSpeedStair.mat"); 
S_neg = Position.signals.values;
avg_neg_S = DataCleaner(S_neg, 5, 0.5, 300, 'Negative Speed Profile');
save('Averaged_Negative_Speed_Profile.mat', 'avg_neg_S');
