T_r1 = 0.15;
T_r2 = 0.25;
T_r3 = 0.5;

omega_zero = 2*pi/T_r1;
A_z = [0,1,0,0,0;
    0,0,1,0,0;
    0,-(omega_zero^2),0,C_c;
    0,0,0,A_c(1,:);
    0,0,0,A_c(2,:)];
B_z = [0;
    0;
    0;
    B_c;]

lambda_c1 = omega_n * exp(1j * (-pi + pi/4));  
lambda_c2 = omega_n * exp(1j * (-pi - pi/4));  
lambda_c3 = omega_n * exp(1j * (-pi + pi/6));  
lambda_c4 = omega_n * exp(1j * (-pi - pi/6));  
lambda_c5 = -omega_n; 

p_z = [lambda_c1,conj(lambda_c1),lambda_c3,conj(lambda_c3),lambda_c5];

K_z = place(A_z,B_z,p_z);
K_chi = K_z(end-1:end);
