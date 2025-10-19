%% Testing for the perfect tracking / rejection using the Extended Estimator Approach:

function [Ae, Be, Ce, Le, p, p_obs, C_rho, K_stateFB_extended] = Controller_StateSpace_Lab1_25_04_25(T_r, Tm, km, N)

    omega_zero = 2*pi*(1/T_r);
    

    A_c = [0  1 ; 0 -1/Tm];
    B_c = [0 ; km/(Tm*N)];
    C_c = [1 0];
    D_c = 0;
    
    % Required Performance Specifications
    ts_ss = 0.15; 
    mp_ss = 0.1; 
    
    % Determining delta and omega_n from desired ts and mp:
    
    delta = log(1/mp_ss) / sqrt(pi^2 + (log(1/mp_ss))^2);
    omega_n = 3 / (delta * ts_ss);
     
    
    % Robust tracking with extended estimator approach
    
    %Tracking of sinusoidal reference + rejection of step disturbance
    
    
    A_rho = [0 1 0; 0 0 1; 0 -omega_zero^2 0];
    C_rho = [1 0 0];
    
    % Extended System Matrices
    Ae = [A_rho zeros(3,2); B_c*C_rho A_c];     % Ae [5x5]
    Be = [0; 0; 0; B_c];                        % Be [5x1] 
    Ce = [0 0 0 C_c];                           % Ce [1x5]
    
    
    % Tentative Pole Allocation
    
    sigma =   -delta*omega_n;
    omega_d = omega_n*sqrt(1-delta^2);
    
    %Controller Pole Alloc. (A - BK)
    
    lambda1_con = sigma + 1i*omega_d;
    
    p = [lambda1_con conj(lambda1_con)]; %given poles
    
    K_stateFB_extended = place(A_c, B_c, p);
    
    %Observer poles (Ae - Le*Ke)
    
    lambda1_obs =  2*omega_n*exp(1i*(-pi+pi/3));
    
    lambda3_obs =  2*omega_n*exp(1i*(-pi+pi/6));
    
    lambda5_obs = -2*omega_n;


    p_obs = [lambda1_obs conj(lambda1_obs) lambda3_obs conj(lambda3_obs) lambda5_obs];
    
    % Using Duality to calculate observer gain
    L_transpose = place(Ae', Ce', p_obs);
    
    % Reverting from dual controller problem to observer problem:
    Le = L_transpose';  

    end





