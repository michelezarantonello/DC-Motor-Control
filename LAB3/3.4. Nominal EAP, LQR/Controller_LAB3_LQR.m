%% State Feedback Design using Static, Scalar LQR

% Approximated system  (prepare for the SRL)
sysG = ss(A_prime, B_prime, C_prime, D_prime);
sysGp = ss(-A_prime, -B_prime, C_prime, D_prime);

%% SRL (Symmetric Root Locus) sketching

%Variables to define the Admissible Region

phi = atan(sqrt(1-delta^2)/delta);

x_max = -3/t_settle;
x_min = -200;

m1 = tan(phi);
m2 = tan(-phi);

x_range = [x_min, x_max];

y_range1 = m1* x_range;
y_range2 = m2* x_range;

%Range of gains to draw a partial root locus
k_range = linspace(2e3,3.5e3, 1e3); 

rlocus(sysG*sysGp,k_range);
hold on;

line(x_range, y_range1, 'Color','b', 'LineWidth',1.5);
line([x_max x_max], [m1*x_max,m2*x_max], 'Color','b', 'LineWidth',1.5);
line(x_range, y_range2, 'Color','b', 'LineWidth',1.5);

xlim([-100 100]);
ylim([-100 100]);

grid on;


K_final = 3.5e3; %Chosen K (smaller K s.t. all the poles are inside the admissible region)
r = 1/K_final; 
K_fb = lqry(sysG, 1, r); %Resulting Optimal State Feedback Gain
