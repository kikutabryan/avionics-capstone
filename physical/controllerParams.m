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
% Values for faster tuning
pitch_rate = deg2rad(20);
pitch_angle = deg2rad(25);
elev_rate = deg2rad(20);
elev_angle = deg2rad(25);
trav_rate = deg2rad(20);
trav_angle = deg2rad(45);

F_max = 1.239;
F_delta = 0.310;
F_max_avail = 1.084;
F_y_max = 0.976;
F_x_max = 0.473;

% Pitch controller
KpPitch = 0.76;
KiPitch = 0;
KdPitch = 0;
maxIntegralPitch = inf;
minIntegralPitch = -inf;
maxPitchRate = pitch_rate;
minPitchRate = -pitch_rate;

% Pitch rate controller
KpPitchRate = 0.84;
KiPitchRate = 0;
KdPitchRate = 0;
maxIntegralPitchRate = F_delta;
minIntegralPitchRate = -F_delta;
maxDeltaForce = F_delta;
minDeltaForce = -F_delta;

% Elevation controller
KpElev = 0.76;
KiElev = 0;
KdElev = 0;
maxIntegralElev = elev_rate;
minIntegralElev = -elev_rate;
maxElevRate = elev_rate;
minElevRate = -elev_rate;

% Elevation rate controller
KpElevRate = 2.66;
KiElevRate = 0;
KdElevRate = 0;
maxIntegralElevRate = F_y_max;
minIntegralElevRate = -F_y_max;
maxElevForce = F_y_max;
minElevForce = -F_y_max;

% Elevation saturation
maxElevCombForce = F_max_avail;
minElevCombForce = 0;

% Travel controller
KpTrav = 0.42;
KiTrav = 0;
KdTrav = 0;
maxIntegralTrav = trav_rate;
minIntegralTrav = -trav_rate;
maxTravRate = trav_rate;
minTravRate = -trav_rate;

% Travel rate controller
KpTravRate = 1.29;
KiTravRate = 0;
KdTravRate = 0;
maxIntegralTravRate = F_x_max;
minIntegralTravRate = -F_x_max;
maxTravForce = F_x_max;
minTravForce = -F_x_max;

% Compensation coefficients
aGravityCompensation = 1.8121;
bGravityCompensation = 1.0325;
cGravityCompensation = 0.1783;

% Tuning reference values
tuningPitchRef = pitch_angle;
tuningElevRef = elev_angle;
tuningTravRef = trav_angle;
tuningPitchRateRef = maxPitchRate;
tuningElevRateRef = maxElevRate;
tuningTravRateRef = maxTravRate;

% Load filter coefficients
filterCoeffs = load('FIR_coefficients.mat');
filterCoeffs = filterCoeffs.b;