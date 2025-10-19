%% SSM Control Design - Robust Tracking using Integral Action

run('Plant_EstimTrialUpdateLAB17_04_25.m'); % defines Tm, km, N, etc.

% Dominant-Pole Accurate Model:

A_c = [0  1 ; 0 -1/Tm];
B_c = [0 ; km/(Tm*N)];
C_c = [1 0];
D_c = 0;


Ae =  [0 C_c; [0;0] A_c];
Be =  [0; B_c];
Ce =  [0 C_c];
De =  0;

r_term = [1 ; 0];

% Determining Nx, Nu by solving for M * [Nx Nu]^T = [0 1]^T
M = [A_c B_c; C_c D_c];
b = [0; 0; 1];

Sol = M\b;

Nx = Sol(1:2, :);
Nu = Sol(3, :);

% Required Performance Specifications
ts_ss = 0.15; 
mp_ss = 0.1; 
 
% Determining delta and omega_n from desired ts and mp:

delta = log(1/mp_ss) / sqrt(pi^2 + (log(1/mp_ss))^2);
omega_n = 3 / (delta * ts_ss);

%The HPF (real derivative):

omega_c = 2*pi*50;
delta_H1 = 1/(sqrt(2));

H1.num = [omega_c^2,0];
H1.den = [1, 2*delta_H1 * omega_c, omega_c^2];

model_name =        'Model_statespacesystem';
pos_ref_name =      'pos_ref_ssm';
pos_meas_name =     'pos_meas_ssm';

function [best_poles, all_results] = TuneIntegralPolePlacement(Ae, Be, model_name, pos_ref_name, pos_meas_name, delta, omega_n)

    % precompute real and imaginary parts for controller poles
    sigma    = -delta * omega_n;
    omega_d  = omega_n * sqrt(1 - delta^2);

    % candidate pole placements
    eigenvalue_sets = {
        [sigma+1i*omega_d, sigma-1i*omega_d, sigma];
        [2*sigma+1i*omega_d, 2*sigma-1i*omega_d, 2*sigma];
        [2*sigma+1i*omega_d, 2*sigma-1i*omega_d, 3*sigma]
    };

    Ncases = numel(eigenvalue_sets);

    % init result table
    all_results = table('Size',[Ncases 3], ...
                        'VariableTypes',{'cell','double','double'}, ...
                        'VariableNames',{'Poles','SettlingTime','Overshoot'});

    % storage for every gain
    K_store = cell(Ncases,1);

    % loop through each candidate
    for i = 1:Ncases
        poles = eigenvalue_sets{i};

        % design state-feedback for augmented system
        K_stateFB = place(Ae, Be, poles);
        K_store{i} = K_stateFB;               % store it
        assignin('base','K_stateFB',K_stateFB);  % push to Simulink

        % simulate
        simOut = sim(model_name,'ReturnWorkspaceOutputs','on');

        % extract measured and reference
        yts = simOut.get(pos_meas_name);
        rts = simOut.get(pos_ref_name);
        t   = yts.Time;
        y   = squeeze(yts.Data);
        r   = interp1(rts.Time, rts.Data, t);

        % step‐response metrics
        S = stepinfo(y, t, r(end), 'SettlingTimeThreshold',0.05);

        % save results
        all_results.Poles{i}        = poles;
        all_results.SettlingTime(i) = S.SettlingTime;
        all_results.Overshoot(i)    = S.Overshoot;
    end

    % pick best: first min Overshoot, then min SettlingTime
    [~, idx_min_os] = min(all_results.Overshoot);
    % among any ties on overshoot, pick smallest settling
    tied = find(all_results.Overshoot == all_results.Overshoot(idx_min_os));
    [~, k] = min(all_results.SettlingTime(tied));
    best_idx = tied(k);

    % return best
    best_poles = eigenvalue_sets{best_idx};
    fprintf('→ Best case (%OS Priority) #%d: Overshoot=%.2f%%, SettlingTime=%.3fs\n', ...
            best_idx, all_results.Overshoot(best_idx), all_results.SettlingTime(best_idx));

    % re-assign the best gain into Simulink
    assignin('base','K_stateFB', K_store{best_idx});

    % display table, highlighting best row
    disp('All candidates:'); 
    disp(all_results);
    disp('Best candidate highlighted below:');
    disp(all_results(best_idx,:));
end


[best_results, full_results]=TuneIntegralPolePlacement(Ae, Be, model_name, pos_ref_name, pos_meas_name, delta, omega_n);
