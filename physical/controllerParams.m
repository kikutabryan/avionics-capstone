clear;
clc;

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
% Thrust curve
aThrust = 0.00268;
bThrust = 0.00111;

% Values for faster tuning
pitchRate = deg2rad(60);
pitchAngle = deg2rad(20);
elevRate = deg2rad(15);
elevAngle = deg2rad(15);
travRate = deg2rad(45);
travAngle = deg2rad(45);

% Force limits
maxF = 1.239;
deltaF = 0.991;
maxFy = 1.239;
maxFx = 0.387;

% Pitch controller
KpPitch = 2.85;
KiPitch = 0;
KdPitch = 0.4;
maxIntegralPitch = inf;
minIntegralPitch = -inf;
maxPitchRate = pitchRate;
minPitchRate = -pitchRate;

% Pitch rate controller
KpPitchRate = 0.90;
KiPitchRate = 0.3;
KdPitchRate = 0.1;
maxIntegralPitchRate = deltaF;
minIntegralPitchRate = -deltaF;
maxDeltaForce = deltaF;
minDeltaForce = -deltaF;

% Elevation controller
KpElev = 0.95;
KiElev = 0;
KdElev = 0;
maxIntegralElev = elevRate;
minIntegralElev = -elevRate;
maxElevRate = elevRate;
minElevRate = -elevRate;

% Elevation rate controller
KpElevRate = 4.27;
KiElevRate = 0.6;
KdElevRate = 0.2;
maxIntegralElevRate = maxFy;
minIntegralElevRate = -maxFy;
maxElevForce = maxFy;
minElevForce = -maxFy;

% Elevation saturation
maxElevCombForce = maxF;
minElevCombForce = 0;

% Travel controller
KpTrav = 0.95;
KiTrav = 0;
KdTrav = 0.2;
maxIntegralTrav = travRate;
minIntegralTrav = -travRate;
maxTravRate = travRate;
minTravRate = -travRate;

% Travel rate controller
KpTravRate = 0.3;
KiTravRate = 0;
KdTravRate = 0;
maxIntegralTravRate = maxFx;
minIntegralTravRate = -maxFx;
maxTravForce = maxFx;
minTravForce = -maxFx;

% Compensation coefficients
aGravityCompensation = 1.8121;
bGravityCompensation = 1.0325;
cGravityCompensation = 0.12;

% Tuning reference values
tuningPitchRef = pitchAngle;
tuningElevRef = elevAngle;
tuningTravRef = travAngle;
tuningPitchRateRef = maxPitchRate;
tuningElevRateRef = maxElevRate;
tuningTravRateRef = maxTravRate;

% Load filter coefficients
posFilterCoeffs = load('posFilterCoeffs.mat');
posFilterCoeffs = posFilterCoeffs.b;

pidFilterCoeffs = load('pidFilterCoeffs.mat');
pidFilterCoeffs = pidFilterCoeffs.b;