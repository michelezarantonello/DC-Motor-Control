%% Bode Method for PID Tuning:
s = tf('s');

% Plant transfer function P(s)(t->h), Motor Torque (\tau_m) to Hub Angle (\theta_h) position:

Dt1 = Jeq*mld.Jb*s^3 + (Jeq*mld.Bb+mld.Jb*Beq)*s^2+(Beq*mld.Bb+mld.k*(Jeq+mld.Jb/gbox.N^2))*s+mld.k*(Beq+mld.Bb/gbox.N^2);
Dt = s * Dt1;

Num = drv.dcgain * mot.Kt *(mld.Jb*s^2 + mld.Bb*s + mld.k);
Den = (gbox.N*s)*((sens.curr.Rs+mot.R)*Dt1 + mot.Kt*mot.Ke*(mld.Jb*s^2+mld.Bb*s+mld.k));

P = Num / Den; 


% PID Controller: C = Dk*e^(j*Dphi), where:

% > Dk = 1/|P(jwc)|
% > Dphi = -pi + phim - arg(P(jwc));

%set the desired settling time and overshoot

ts = 0.85; %settling time at 5% in seconds
mp = 0.3; %maximum overshoot (0.3 means 30%)

%the damping factor delta
delta = log(1/mp)/sqrt(pi^2 + log(1/mp)^2);

% Desired wc (crossover frequency) and phim (phase margin)
wc = 3 / (ts * delta);

%phim = 1.04 - 0.8*mp;
phim = atan(2*delta/sqrt(sqrt(1+4*delta^2)-2*delta^2));

% Plant Gain (Gm) and Phase (Pm) at crossover frequency wc: 
[Gm, Pm] = bode(P, wc);  %gives Gain Margin and Phase Margin [deg] 



% Controller magnitude:
% Dk = 1/|P(jwc)|

Dk = 1/Gm;

% Plant phase:
ArgP = Pm/360 * 2 * pi;

% Controller phase:
Dphi = -pi + phim - ArgP;

% Alpha and Beta terms (from the 'Best Results' file - Riccardo's Tests)

a = 4;   %alpha initially set as 4
b = 0.1;    %beta between 2 and 5, to implement the real derivative (causal)

%System of gains to be solved:

Kp = Dk*cos(Dphi);
Td = (tan(Dphi) + sqrt(tan(Dphi)^2 + 4*a))/(2*wc);
Ti = a * Td;

% TL as beta times wc
TL = b / wc;

Ki = Kp/Ti; %Integral Gain
Kd = Kp*Td; %Derivative Gain

% C in terms of PID Parameters:
% C = Kp*(1 + 1/(Ti*s) + Td*s/(TL*s+1));


% For the Anti-Windup Implementation
Tw = ts/5;
Kw = 1/Tw;  %Antiwindup Gain

%% Optimized challenge values w/ Anti_windup

%settling time 0.2121 for ref = 50°
Kp = 32.3; %32.3
Ki = 98; %98
Kd = 1.84; %1.84

%% Normal values

%

% w/o AW: settling time 1.02 overshoot 45
% w/ AW: settling time 0.75 overshoot 30