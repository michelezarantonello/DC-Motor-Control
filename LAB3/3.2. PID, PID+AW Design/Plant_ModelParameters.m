%% Conversion gains

rpm2rads = 2 * pi/60; % [rpm] −> [rad/s] 
rads2rpm = 60/2/pi; % [rad/s] −> [rpm]
rpm2degs = 360/60; % [rpm] −> [deg/s]
degs2rpm = 60/360; % [deg/s] −> [rpm]
deg2rad = pi/180; % [deg] −> [rad]
rad2deg = 180/pi; % [rad] −> [deg]
ozin2Nm = 0.706e-2; % [oz * inch] −> [N * m]



%% DC motor nominal parameters
% brushed DC−motor Faulhaber 2338S006S

mot.R = 2.6; % armature resistance
mot.L = 180e-6; % armature inductance
mot.Kt = 1.088 * ozin2Nm; % torque constant
mot.Ke = 0.804e-3 * rads2rpm; % back−EMF constant
mot.J = 5.523e-5 * ozin2Nm; % rotor inertia
mot.B = 0.0; % viscous friction coeff (n.a.)
mot.eta = 0.69; % motor efficiency
mot.PN = 3.23/mot.eta; % nominal output power
mot.UN = 6; % nominal voltage
mot.IN = mot.PN/mot.UN; % nominal current
mot.tauN = mot.Kt * mot.IN; % nominal torque
mot.taus = 2.42 * ozin2Nm; % stall torque
mot.w0 = 7200 * rpm2rads; % no−load speed


%% Gearbox nominal parameters

% planetary gearbox Micromotor SA 23/1
gbox.N1 = 14; % 1st reduction ratio (planetary gearbox)
gbox.eta1 = 0.80; % gearbox efficiency

% external transmission gears
gbox.N2 = 1; % 2nd reduction ratio (external trasmission gears)
gbox.J72 = 1.4e-6; % inertia of a single external 72 tooth gear
gbox.eta2 = 1; % external trasmission efficiency (n.a.)

% overall gearbox data
gbox.N = gbox.N1 * gbox.N2; % total reduction ratio
gbox.eta = gbox.eta1 * gbox.eta2; % total efficiency
gbox.J = 3 * gbox.J72; % total inertia (at gearbox output)


%% Mechanical load (mld) nominal parameters

% hub params
mld.Jh = 6.84e-4; % moment of inertia
mld.Bh = 2.5e-4; % viscous friction coeff
mld.tausf = 1.0e-2; % static friction (estimated)

% beam & elastic joint params
mld.Jb = 1.4e-3; % moment of inertia
mld.d = 5.0e-2; % flex joint damping coeff (estimated)
mld.wn = 24.5; % flex joint natural freq (estimated)
mld.Bb = mld.Jb * 2 * mld.d * mld.wn; % beam viscous friction
mld.k = mld.Jb * mld.wn^2; % flex joint stiffness


%% Voltage driver nominal parameters

% op−amp circuit params
drv.R1 = 7.5e3; % op−amp input resistor (dac to non−inverting in)
drv.R2 = 1.6e3; % op−amp input resistor (non−inverting in to gnd)
drv.R3 = 1.2e3; % op−amp feedback resistor (output to inverting in)
drv.R4 = 0.5e3; % op−amp feedback resistor (inverting in to gnd)
drv.C1 = 100e-9; % op−amp input capacitor
drv.outmax = 12; % op−amp max output voltage

% voltage driver dc−gain
drv.dcgain = drv.R2/(drv.R1+drv.R2) * (1 + drv.R3/drv.R4);

% voltage driver time constant
drv.Tc = drv.C1 * drv.R1 * drv.R2/(drv.R1+drv.R2);


%% Sensors data 76

% shunt resistor
sens.curr.Rs = 0.5;

% Hewlett−Packard HEDS−5540#A06 optical encoder
sens.enc.ppr = 500 * 4; % pulses per rotation
sens.enc.pulse2deg = 360/sens.enc.ppr; % [pulses] −> [deg]
sens.enc.pulse2rad = 2 * pi/sens.enc.ppr; % [pulses] −> [rad]
sens.enc.deg2pulse = sens.enc.ppr/360; % [deg] −> [pulses]
sens.enc.rad2pulse = sens.enc.ppr/2/pi; % [rad] −> [pulses]

% potentiometer 1 (Spectrol 138−0−0−103) − installed on motor box
sens.pot1.range.R = 10e3; % ohmic value range
sens.pot1.range.V = 5; % voltage range
sens.pot1.range.th_deg = 345; % angle range [deg]
sens.pot1.range.th = sens.pot1.range.th_deg * deg2rad; % angle range [rad]
sens.pot1.deg2V = sens.pot1.range.V / sens.pot1.range.th_deg; % sensitivity [V/deg]
sens.pot1.rad2V = sens.pot1.range.V / sens.pot1.range.th; % sensitivity [V/rad]
sens.pot1.V2deg = 1/sens.pot1.deg2V; % conversion gain [V] −> [deg]
sens.pot1.V2rad = 1/sens.pot1.rad2V; % conversion gain [V] −> [rad]

% potentiometer 2 (Spectrol 357−0−0−103) − installed on hub
sens.pot2.range.R = 10e3; % ohmic value range
sens.pot2.range.V = 5; % voltage range
sens.pot2.range.th_deg = 340; % angle range [deg]
sens.pot2.range.th = sens.pot2.range.th_deg * deg2rad; % angle range [rad]
sens.pot2.deg2V = sens.pot2.range.V / sens.pot2.range.th_deg; % sensitivity [V/deg]
sens.pot2.rad2V = sens.pot2.range.V / sens.pot2.range.th; % sensitivity [V/rad]
sens.pot2.V2deg = 1/sens.pot2.deg2V; % conversion gain [V] −> [deg]
sens.pot2.V2rad = 1/sens.pot2.rad2V; % conversion gain [V] −> [rad]
sens.pot2.noise.var = 3.5e-7; % out noise variance [V^2]


%% Data acquisition board (daq) data

% NI PCI−6221 DAC data
daq.dac.bits = 16; % resolution (bits)
daq.dac.fs = 10; % full scale
daq.dac.q = 2 * daq.dac.fs/(2^daq.dac.bits-1); % quantization

% NI PCI−6221 ADC data
daq.adc.bits = 16; % resolution (bits)
daq.adc.fs = 10; % full scale (as set in SLDRT Analog Input block)
daq.adc.q = 2 * daq.adc.fs/(2^daq.adc.bits-1); % quantization

%% Calculation of Jeq with hub parameters

Jeq = mot.J + (mld.Jh+gbox.J)/gbox.N^2;

%% Added parameters and values to make Simulink work

pulse2deg = 360/2000;

% DAC & ADC
Ts = 0.001; % Sampling time for DAC, ADC, etc

% previously estimated
Beq = 1.0459e-6; % viscous friction motor
tau_sf = 0.0061; 

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
