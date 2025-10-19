%% State Feedback Design using Static, Matrix-Valued LQR (Robust + Integral)

%% Weight / Cost Allocation according to Bryson's Rule:

%cost_i = (1/[deviation_i]^2)
%|theta_h - theta_h*| < (0.3)theta_h* [deg] ; |theta_d| < 5 [deg]; |u|< 10V; 
%q1 = 1/((0.3)theta_h*)^2 ; q2 = 1/(theta_h*)^2; R = r1 = 1/(u)^2;

% q1 is now q_11 (the Integral Cost): It cannot be 0 because in that case,
% the pair (Ae, sqrt(Qe)) is not Detectable, and the A.R.E. has no Solution


function [K_fb] = LAB3_LQR_Matrix(sysG, q1_value)

    q1 = q1_value;
    q2 = 1/((0.3)*(5))^2;
    q3 = 1/(5)^2;
    u1 = 1/(10)^2;
    
    Q_vec = [q1; q2; q3; 0; 0];
    Q = diag(Q_vec);
    
    R = u1;
    
    K_fb = lqr(sysG, Q, R); %Resulting Optimal State Feedback Gain

end    
