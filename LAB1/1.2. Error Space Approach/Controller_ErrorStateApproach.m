function [K_chi,K_z, omega_input] = ErrorStateApproach(T_r_input, Tm, km, N)

%% Testing for the perfect tracking / rejection using the Error Space Approach:

% Original DC Motor SSM Model
A_c = [0  1; 0 -1/Tm];
B_c = [0 ; km/(Tm*N)];
C_c = [1 0];
D_c = 0;

% Required Performance Specifications
ts_ss = 0.15; 
mp_ss = 0.1; 
 
% Determining delta and omega_n from desired ts and mp:
delta = log(1/mp_ss) / sqrt(pi^2 + (log(1/mp_ss))^2);
omega_n = 3 / (delta * ts_ss);


% Use the input T_r to define the working frequency
omega_input = 2*pi/T_r_input;

% The sinusoidal+step reference creates an error exosystem (subspace of A_z)
% of dimension 3, and the original SSM is of dimension 2, making 
% dim(A_z) =5

A_z = [0,1,0,0,0;
       0,0,1,0,0;
       0,-(omega_input^2),0,C_c; % use omega_input here
       0,0,0,A_c(1,:);   % row 1 of original SSM
       0,0,0,A_c(2,:)];  % row 2 of original SSM


B_z = [0;
       0;
       0;
       B_c;];

% Desired eigenvalue / pole allocation:
lambda_c12 = omega_n * exp(1j * (-pi + pi/4));  
lambda_c34 = omega_n * exp(1j * (-pi + pi/6));  
lambda_c5 = -omega_n; 

p_z = [lambda_c12,conj(lambda_c12),lambda_c34,conj(lambda_c34),lambda_c5];

% Designing the (Error-Space Extended) State Feedback
K_z = place(A_z,B_z,p_z);

% K_z(1:3) contains the Error-Subspace Feedforward (numerator of H(s))
% K_z(4:5) contains the Original SSM feedback matrix := K_chi

% Extracting K_chi := original SSM Feedback
K_chi = K_z(end-1:end);

end

