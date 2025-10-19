%% System parameters (Electrical/Motor/Mechanical (Load) side):
kdrv = 0.6;
ke = 7.68 * 0.001;
Ra = 2.6;
Rs = 0.5;
Jm = 3.9 * 0.0000001;
Jd = 3 * 0.00001;
J72 = 1.4 * 0.000001;
N = 14;

Bm = 0;
Bl = 0;

%% Estimated Parameters

%from B_eq_avg in 'Parameter Estimation'
Beq = 1.0459e-06;

%from tau_sf_avg in 'Parameter Estimation'
tausf = 0.0061;

%from J_eq_avg in 'Parameter Estimation'
Jeq = 9.8651e-07;

% Conversion units:
pulse2deg = 360/2000;
deg2rad = 2*pi/360;
rad2deg = 180/pi;   % [rad] −> [deg]
rads2rpm = 60/2/pi; % [rad/s] −> [rpm]
rpm2rads = 2*pi/60;
degs2rpm = 60/360;
ozin2Nm = 0.706e-2;

Kt = 1.088 * ozin2Nm;

% Sampling Time (for the ZOH)
Ts = 1e-3;

%Voltage Driver Parameters

drv.R1 = 7.5e3; % op−amp input resistor (dac to non−inverting in)
drv.R2 = 1.6e3; % op−amp input resistor (non−inverting in to gnd)
drv.R3 = 1.2e3; % op−amp feedback resistor (output to inverting in)
drv.R4 = 0.5e3; % op−amp feedback resistor (inverting in to gnd)
drv.C1 = 0.0000001; % op−amp input capacitor
drv.outmax = 12; % op−amp max output voltage
drv.Tc = drv.C1 * drv.R1*drv.R2/(drv.R1+drv.R2);
L = 180e-6;

% Equivalent Parameters:
Jl = Jd + 3*J72;

Req = Ra + Rs;

km = kdrv*Kt/(Req*Beq + Kt*ke);

Tm = Req*Jeq/(Req*Beq + Kt*ke);


%% Bode Method for PID Tuning:
s = tf('s');

% Dominant Pole Plant Approximation:
P = (km / (Tm*s + 1))*1/(N*s);

% PID Controller: C = Dk*e^(j*Dphi), where:
% > Dk = 1/|P(jwc)|
% > Dphi = -pi + phim - arg(P(jwc));


ts = 0.15; %settling time at 5% in seconds 0.15
mp = 0.1; %maximum overshoot  0.1 = 10% of amplitude

%the damping factor delta
delta = log(1/mp)/sqrt(pi^2 + log(1/mp)^2);

% Desired wc (crossover frequency) and phim (phase margin)
wc = 3 / (ts * delta);
phim = 1.04 - 0.8*mp;

% Plant Gain (Gm) and Phase (Pm) at crossover frequency wc: 
[Gm, Pm] = bode(P, wc);

% Controller magnitude:
% Dk = 1/|P(jwc)|
Dk = 1/Gm;

% Plant phase:
ArgP = Pm*(2*pi)/360;

% Controller phase:
Dphi = -pi + phim - ArgP;

% Alpha and Beta terms (from the 'Best Results' file - Riccardo's Tests)

a = 40;   %alpha initially set as 4
b = 5;    %beta between 2 and 5, to implement the real derivative (causal)

%System of gains to be solved:

Kp = Dk*cos(Dphi);
Td = (tan(Dphi) + sqrt(tan(Dphi)^2 + 4*a))/(2*wc);
Ti = a * Td;

% TL as beta times wc
TL = b * wc;

Ki = Kp/Ti; %Integral Gain
Kd = Kp*Td; %Derivative Gain

% C in terms of PID Parameters:

C = Kp*(1 + 1/(Ti*s) + Td*s/(TL*s+1));



%% Parameter Estimation: Creating Reference Signals and Filtering

% High-Pass Filter to measure the motor speed
Hw.w = 2*pi*20;
Hw.delta = 1/sqrt(2);
Hw.num = [Hw.w^2 0];
Hw.den = [1 2*Hw.delta*Hw.w Hw.w^2];

% Low-Pass Filter to measure the current
Hi.w = Hw.w;
Hi.delta = Hw.delta;
Hi.num = [Hi.w^2];
Hi.den = [1 2*Hi.delta*Hi.w Hi.w^2];

Dw = 50; %Amplitude of speed step reference (delta omega)
A = 450; %Amplitude of acceleration square wave to test

%% For the Anti-Windup Implementation
Tw = 7/5;
Kw = 1/Tw;  %Antiwindup Gain