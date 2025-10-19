function isDetectable = check_detectability(Ae, Qe)

    % Check dimensions match
    if size(Ae,1) ~= size(Qe,1)
        error('Dimension mismatch between Ae and Qe.');
    end

    % Compute matrix square root of Qe (assumes symmetric PSD)
    [V, D] = eig(Qe);
    D_sqrt = sqrt(max(D, 0));  % ensure numerical stability
    Q_sqrt = V * D_sqrt * V';

    % Define C_Q as Q_sqrt (treat rows as outputs)
    Cq = Q_sqrt;

    % Build Observability Matrix
    Ob = obsv(Ae, Cq);
    rank_Ob = rank(Ob);

    fprintf('\n--- Detectability Check ---\n');
    fprintf('Observability Matrix Rank: %d / %d\n', rank_Ob, size(Ae,1));

    if rank_Ob == size(Ae,1)
        disp('✅ The system (Ae, sqrt(Qe)) is observable, thus detectable.');
        isDetectable = true;
        return;
    end

    % If not observable, check detectability:
    % Check whether all unobservable modes have negative real parts
    % i.e., are asymptotically stable

    % Find null space of observability matrix (unobservable subspace)
    null_obs = null(Ob);
    eigs_Ae = eig(Ae);

    % Project eigenvectors onto null space
    unobs_modes = [];
    for i = 1:length(eigs_Ae)
        % Solve (Ae - lambda*I)v = 0 for each eigenvalue
        lambda = eigs_Ae(i);
        [V_lambda, ~] = eig(Ae - lambda * eye(size(Ae)));
        V_lambda = V_lambda(:, 1); % take one eigenvector

        if norm(Ob * V_lambda) < 1e-6  % approx unobservable
            unobs_modes(end+1) = lambda; %#ok<AGROW>
        end
    end

    if all(real(unobs_modes) < 0)
        disp('✅ System is detectable: all unobservable modes are stable.');
        isDetectable = true;
    else
        disp('❌ System is NOT detectable: there are unstable unobservable modes.');
        disp('Unobservable unstable modes:');
        disp(unobs_modes(real(unobs_modes) >= 0));
        isDetectable = false;
    end
end


% Running a check for the case where q_11 = 0 (Loss of Detectability)

q1 = 0;

q2 = 1/((0.3)*(5))^2;
q3 = 1/(5)^2;
u1 = 1/(10)^2;
    
Q_vec = [q1; q2; q3; 0; 0];
Q = diag(Q_vec);

check_detectability(Ae, Q)