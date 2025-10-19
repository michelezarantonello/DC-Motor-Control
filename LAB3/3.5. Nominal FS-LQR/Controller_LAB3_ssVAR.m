% Equivalent Resistance
Req = sens.curr.Rs + mot.R;


% omega_c and delta1 for the HPF
omega_c = 2 * pi * 50; 
delta1 = 1 / sqrt(2);

% HPF for the Real Derivative
H1_num = [omega_c^2 0];  
H1_den = [1, 2*delta1*omega_c, omega_c^2];

%% SSM Matrices:
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     -mld.k / (gbox.N^2 * Jeq), mld.k / (gbox.N^2 * Jeq), (-1 / Jeq) * (Beq + (mot.Kt * mot.Ke) / Req), 0;
     mld.k / mld.Jb, -mld.k / mld.Jb, 0, -mld.Bb / mld.Jb];

B = [0;
     0;
     mot.Kt * drv.dcgain / (gbox.N * Jeq * Req);
     0];

Bd = [0;
      0;
      -1 / (gbox.N^2 * Jeq);
      0];

%% Transformation Matrix to redefine the state in terms of theta_h and
% theta_d only [Hub Point-of-View CoB]:

T = [1 0 0 0; 
     1 1 0 0;
     0 0 1 0;
     0 0 1 1];

% Post - CoB Matrices:

A_prime = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, mld.k/(gbox.N^2*Jeq), (-1/Jeq)*(Beq+(mot.Kt*mot.Ke)/Req), 0;
    0,-mld.k/mld.Jb-mld.k/(Jeq*gbox.N^2),-mld.Bb/mld.Jb+(1/Jeq)*(Beq+(mot.Kt*mot.Ke)/Req),-mld.Bb/mld.Jb];


B_prime = [
    0;
    0;
    mot.Kt * drv.dcgain / (gbox.N * Jeq * Req);
    -mot.Kt * drv.dcgain / (gbox.N * Jeq * Req)
];

C_prime = [1,0,0,0];

D_prime = zeros(1,1);

B_prime_d = [
    0;
    0;
    -1 / (gbox.N^2 * Jeq);
    1 / (gbox.N^2 * Jeq)
];

%% Performance Specifications:
t_settle = 0.85;
Mp = 0.3;


delta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)*log(1/Mp));
phi = atan(sqrt(1 - delta^2)/ delta);
omega_n = 3 / (delta*t_settle);

%% Eigenvalue Placement:
lambda_1 = omega_n * exp(1j * (-pi + phi));     
lambda_3 = omega_n * exp(1j * (-pi + phi/2)); 
lambda_2 = conj(lambda_1);     
lambda_4 = conj(lambda_3);

p = [lambda_1 lambda_2 lambda_3 lambda_4];

K_fb = place(A_prime,B_prime,p);


%% Solving for the Nominal Feedforward Gains:
M = [A_prime,B_prime;C_prime,D_prime];

Sol = inv(M) * [0;0;0;0;1];

Nx = Sol(1:4, :);
Nu = Sol(5,:);


%K_fb = [59.16,17.91,2.60,1.04];