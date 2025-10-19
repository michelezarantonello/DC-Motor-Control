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

%% For the Feedforward Implementation

% Estimated Parameters:

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
