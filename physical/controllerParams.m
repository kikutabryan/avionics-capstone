% PLEASE SET THE HELICOPTER NUMBER AND Vsum
heli = 1;

% VSUM - FOR LAB 3 ONLY
Vsum = 26.3;

%-----[HELI SET VARIABLES]--------
% THESE VARIABLE ARE SPECIFIC TO EACH HELICOPTER
TRAV_SS = 90;
ELEV_START = -24.50;
ELEV_SLF = 24.50;            % The elevation at SLF [deg]
K_CABLE = 3;
VSUM_MAX = 30;
U_MAX = (VSUM_MAX - Vsum)*0.5;

%-----[HELICOPTER CONSTANTS]---------------
% THESE APPLY TO ALL OF THE HELICOPTERS
KE_CNT = 4096;
KE_C2D = -360/4096;
D2R = pi/180.0;
R2D = 180/pi;
K_DAQ = 10;

%-----[CONTROLLER CONSTANTS]---------------
% Pitch controller
KpPitch = 1.11;
KiPitch = 0;
KdPitch = 0.3;
maxIntegralPitch = inf;
minIntegralPitch = -inf;
maxPitchRate = deg2rad(35);
minPitchRate = -deg2rad(35);

% Pitch rate controller
KpPitchRate = 0.78;
KiPitchRate = 0.2;
KdPitchRate = 0.2;
maxIntegralPitchRate = inf;
minIntegralPitchRate = -inf;
maxDeltaForce = 0.5;
minDeltaForce = -0.5;

% Elevation controller
KpElev = 0.76;
KiElev = 0;
KdElev = 0;
maxIntegralElev = inf;
minIntegralElev = -inf;
maxElevRate = deg2rad(20);
minElevRate = -deg2rad(20);

% Elevation rate controller
KpElevRate = 5.99;
KiElevRate = 0.2;
KdElevRate = 0.5;
maxIntegralElevRate = inf;
minIntegralElevRate = -inf;
maxElevForce = 2;
minElevForce = -2;

% Elevation saturation
maxElevCombForce = 2;
minElevCombForce = 0;

% Travel controller
KpTrav = 0.42;
KiTrav = 0;
KdTrav = 0;
maxIntegralTrav = inf;
minIntegralTrav = -inf;
maxTravRate = deg2rad(20);
minTravRate = -deg2rad(20);

% Travel rate controller
KpTravRate = 2.49;
KiTravRate = 0;
KdTravRate = 0;
maxIntegralTravRate = inf;
minIntegralTravRate = -inf;
maxTravForce = 0.9;
minTravForce = -0.9;

% Compensation coefficients
aGravityCompensation = 3.5837;
bGravityCompensation = 2.042;
cGravityCompensation = 0.3;

% Tuning reference values
tuningPitchRef = deg2rad(30);
tuningElevRef = deg2rad(24.5);
tuningTravRef = deg2rad(45);
tuningPitchRateRef = maxPitchRate;
tuningElevRateRef = maxElevRate;
tuningTravRateRef = maxTravRate;

% Load filter coefficients
filterCoeffs = load('FIR_coefficients.mat');
filterCoeffs = filterCoeffs.b;