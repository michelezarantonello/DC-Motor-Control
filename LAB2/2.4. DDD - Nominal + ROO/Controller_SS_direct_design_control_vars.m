%% Direct Digital Design [D³]: SSM Setup

function [rad2deg, deg2rad, Nu, Nx, K_stateFB, sigmaD_tf_num, sigmaD_tf_den, Phi_o, Gamma_o, H_o, J_o, Ts] = SS_direct_design_control_vars(Ts)


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


% Performance Requirements:
ts_ss = 0.15; 
mp_ss = 0.1; 

delta = log(1/mp_ss) / sqrt(pi^2 + (log(1/mp_ss))^2);
omega_n = 3 / (delta * ts_ss);

% Controller Eigenvalues:
lambda1 = -delta * omega_n + 1i * omega_n * sqrt(1 - delta^2);
lambda2 = -delta * omega_n - 1i * omega_n * sqrt(1 - delta^2);

% Dominant-Pole Motor Approximation
A_c = [0,1; 0, -1/Tm];
B_c = [0;km/(Tm*N)];
C_c = [1,0];
D_c = [0];

sigmaC = ss(A_c, B_c, C_c, D_c); % CT SSM

sigmaD = c2d(sigmaC, Ts, 'zoh'); % Discretized SSM
sigmaD_tf = tf(sigmaD);

[sigmaD_tf_num,sigmaD_tf_den] = tfdata(sigmaD_tf, 'v');

%% Discrete mapping of controller eigs. using z = e^(sT)

discrete_lambda1 = exp(lambda1*Ts);
discrete_lambda2 = exp(lambda2*Ts);

% DT Observer:

[phi11,phi12,phi21,phi22] = deal(sigmaD.A(1,1), sigmaD.A(1,2), sigmaD.A(2,1), sigmaD.A(2,2));

[gamma1,gamma2] = deal(sigmaD.B(1,1), sigmaD.B(2,1));

ampl = -5; %observer poles to have Real Part 5x greater than controller poles

p_obs = exp(ampl*abs(lambda1)*Ts); 
L_obs = place(phi22,phi12,p_obs); %estimator gain

%Matrices of the SS reduced order observer
Phi_o = phi22 - L_obs * phi12; %(A)
Gamma_o = [gamma2-L_obs*gamma1, (phi22-L_obs*phi12)*L_obs+phi21-L_obs*phi11]; %(B)
H_o = [0;1]; %(C)
J_o = [0, 1; 0, L_obs]; %(D)

%%  Determining Nx and Nu FF Gains (for Nominal Design)
M = [sigmaD.A-eye(2), sigmaD.B; sigmaD.C, 0];
solution = inv(M)*[0;0;1];
Nx = solution(1:2,:);
Nu = solution(3,:);
K_stateFB = place(sigmaD.A, sigmaD.B, [discrete_lambda1,discrete_lambda2]);


end







