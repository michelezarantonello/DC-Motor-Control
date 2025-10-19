%% Direct Digital Design [D³]: SSM Setup (Robust+Integral+ROO)

function [rad2deg, deg2rad, Nu, Nx, K_stateFB, K_I, sigmaD_tf_num, sigmaD_tf_den, Phi_o, Gamma_o, H_o, J_o, Ts] = SS_direct_design_control_vars_robusttracking(Ts)

% Estimated Parameters:
Beq = 1e-6;
tausf = 9e-3;
Jeq = 6e-7;


% Motor and Conversion Parameters:
Tm = 0.029955438675947;
km = 74.224552255736010;
N = 14;
deg2rad = 2*pi/360;
rad2deg = 180/pi;   % [rad] −> [deg]
rpm2rads = 2*pi/60;


% Dominant-Pole Motor Approximation
A_c = [0,1; 0, -1/Tm];
B_c = [0;km/(Tm*N)];
C_c = [1,0];
D_c = [0];

sigmaC = ss(A_c, B_c, C_c, D_c); %CT SSM
sigmaD = c2d(sigmaC, Ts, 'zoh'); %Discretized SSM


aliasingCheck(Ts,sigmaD);

sigmaD_tf = tf(sigmaD);

%turn the SSM into a TF
[sigmaD_tf_num, sigmaD_tf_den] = tfdata(sigmaD_tf, 'v');

Phi = sigmaD.A;
Gamma = sigmaD.B;
H = sigmaD.C;
J = sigmaD.D;

%Original Controller Poles:

% Performance Specifications:

ts_ss = 0.15; %settling time state space
mp_ss = 0.1;  %max overshoot state space

delta = log(1/mp_ss) / sqrt(pi^2 + (log(1/mp_ss))^2);
omega_n = 3 / (delta * ts_ss);

sigma = -delta*omega_n;
omega_d = omega_n*sqrt(1-delta^2);

% Set of candidate poles (1 of the 4 cases) taken from LAB1

lambda1_1 = sigma+1i*omega_d;
lambda1_2 = sigma-1i*omega_d;
lambda1_3 = sigma;

%Discretized poles using the (z <- s) mapping
discrete_lambda1 = exp(lambda1_1 * Ts);
discrete_lambda2 = exp(lambda1_2 * Ts);
discrete_lambda3 = exp(lambda1_3 * Ts);

%Augmented SSM: SigmaE
Phi_e = [1, H; [0;0], Phi];
Gamma_e = [0; Gamma];
p = [discrete_lambda1, discrete_lambda2, discrete_lambda3];

% Extended State Feedback Law
K_stateFB_extended = place(Phi_e, Gamma_e, p);

% Separating the Feedback Gains
K_stateFB = K_stateFB_extended(:,2:3);
K_I = K_stateFB_extended(:,1);

% ROO Setup:

[phi11,phi12,phi21,phi22] = deal(sigmaD.A(1,1), sigmaD.A(1,2), sigmaD.A(2,1), sigmaD.A(2,2));
[gamma1,gamma2] = deal(sigmaD.B(1,1), sigmaD.B(2,1));

ampl = -5; 

p_obs = exp(ampl*abs(lambda1_1)*Ts); %discrete pole

L_obs = place(phi22,phi12,p_obs); %estimator gain

%Matrices of the SS ROO:
Phi_o = phi22 - L_obs * phi12; %(A)
Gamma_o = [gamma2-L_obs*gamma1, (phi22-L_obs*phi12)*L_obs+phi21-L_obs*phi11]; %(B)
H_o = [0;1]; %(C)
J_o = [0, 1; 0, L_obs]; %(D)

% Calculating the Feedforward gains:

M = [sigmaD.A-eye(2), sigmaD.B; sigmaD.C, 0];
solution = inv(M)*[0;0;1];
Nx = solution(1:2,:);
Nu = solution(3,:);


end

