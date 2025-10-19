%% Setting up Matrix Plant Representation

theta = 0;      % ATTENTION! I PUT THIS HERE AS A PLACEHOLDER VARIABLE IN THE CALCULATIONS
theta_dot = 0;  % ATTENTION! I PUT THIS HERE AS A PLACEHOLDER VARIABLE IN THE CALCULATIONS

%% Parameters, Variables, Vectors, Matrices that need to be calculated
% Each relevant expression has its Index Number (..) as in METHODS Paper

% ~ Gearbox viscous friction coefficient (at output shaft is gbox.B
% ~ Wheel viscous friction coefficient is wheel.B

M11 = 2*wheel.Iyy + 2 * gbox.N^2*mot.rot.Iyy + ...
    (body.m + 2*wheel.m + 2*mot.rot.m)*wheel.r^2; % (36) I've used wheel.r radius as r...

% M12 = M21 are not used in simulink, I copied them in the Fcn blocks, they
% are then written here for completion and availability

M12 = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy + ...
    (body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r*cos(theta); % (36) l := zbb = body.zb, ie CoM z-coord wrt body frame

M21 = M12;

M22 = body.Iyy + 2*(1-gbox.N)^2 *mot.rot.Iyy + ...
    body.m*body.zb^2 + 2*mot.rot.m*mot.rot.zb^2; % (36)

M = [M11 M12; M21 M22];

% All of the following C Matrices
% are not used in simulink, I copied them in the Fcn blocks, they
% are then written here for completion and availability

C11 = 0; % (37)
C12 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r*sin(theta)*theta_dot;  % (37)
C21 = 0; % (37)
C22 = 0; % (37)

C = [C11 C12; C21 C22];

Fv = [2*(gbox.B + wheel.B) -2*gbox.B; -2*gbox.B 2*gbox.B]; % (38)

% All of the following g Matrices
% are not used in simulink, I copied them in the Fcn blocks, they
% are then written here for completion and availability

g1 = 0; % (39)
g2 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*g*sin(theta); % (39)

g12 = [g1; g2]; % (39)


ua2tau = (2 * gbox.N * mot.Kt / mot.R) * [1; -1]; % (55)
Fv1 = Fv + (2 * gbox.N^2 * mot.Kt * mot.Ke/ mot.R)*[1 -1; -1 1]; % (55)

%% Tilt estimation parameters and calculations

s = tf('s');

% [Hz] tentative cut-off frequency for LPF of the til estimation subsystem

%fc = 0.0005; % for 1st order LPF 

fc = 0.0001;  % for 2nd order LPF 

%fc = 0.0005; % for 3rd order LPF

Tc = 1/(2*pi*fc);

%LPFcontinuous = 1 / (Tc * s + 1); % first order version
LPFcontinuous = (2*Tc * s + 1) / (Tc * s + 1)^2; % second order version
%LPFcontinuous = (3*Tc^2 * s^2 + 3*Tc * s + 1) / (Tc * s + 1)^3; % third order version

LPF = c2d(LPFcontinuous,Ts);

% "LPF.Numerator" or "LPF.Numerator(1) returns a 1x1 cell containing an array with the Num values
% so it's sufficient to use "LPF.Numerator{1}" to extract the CONTENT of
% the cell

LPFNum = LPF.Numerator{1};
LPFDen = LPF.Denominator{1};

%% Simple state observer

% Filter for speed estimation, as seen in (7), Page 12 in METHODS paper

N_forHwz = 3; % as suggested in (7)
z = tf('z', Ts);

Hwz = (1 - z^(-N_forHwz)) / (N_forHwz*Ts);

HwzNum = Hwz.Numerator{1};
HwzDen = Hwz.Denominator{1};

