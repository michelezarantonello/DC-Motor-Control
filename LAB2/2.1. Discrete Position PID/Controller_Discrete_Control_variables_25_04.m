%General variables required
ts = 0.15;
mp = 0.1;
pulse2deg = 360/2000;
deg2rad = 2*pi/360;

%Original continuous time PID controller results (Riccardo)
wc = 30.263227570050770;

Kp = 8.456047228584202;
Td = 0.007998940272146;
Ti = 0.319957610885822;
TL = 1/(2*wc);
Ki = Kp/Ti;
Kd = Kp*Td;

s = tf('s');
Cc = Kp*(1 + 1/(Ti*s) + Td*s/(TL*s+1));


%Digital controller test
T1 = 1e-3;
T2 = 10e-3;
T3 = 50e-3;

% Sampling times
Ts_values = [T1, T2, T3];

%Anti-Windup Implementation
Tw = 7/5;
Kw = 1/Tw;  %Antiwindup Gain

%Discretization of PID controller (tustin)
Cd = c2d(Cc, Ts, 'tustin');
[Cd_num, Cd_den] = tfdata(Cd, 'v');