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
KpPitch = 0.42;
KiPitch = 0;
KdPitch = 0;
maxIntegralPitch = inf;
minIntegralPitch = -inf;
maxPitchRate = deg2rad(20);
minPitchRate = -deg2rad(20);

% Pitch rate controller
KpPitchRate = 1.36;
KiPitchRate = 0;
KdPitchRate = 0;
maxIntegralPitchRate = inf;
minIntegralPitchRate = -inf;
maxDeltaForce = 0.5;
minDeltaForce = -0.5;

% Elevation controller
KpElev = 0.38;
KiElev = 0;
KdElev = 0;
maxIntegralElev = inf;
minIntegralElev = -inf;
maxElevRate = deg2rad(10);
minElevRate = -deg2rad(10);

% Elevation rate controller
KpElevRate = 10.89;
KiElevRate = 0;
KdElevRate = 0;
maxIntegralElevRate = inf;
minIntegralElevRate = -inf;
maxElevForce = 2;
minElevForce = 0;

% Travel controller
KpTrav = 0.42;
KiTrav = 0;
KdTrav = 0;
maxIntegralTrav = inf;
minIntegralTrav = -inf;
maxTravRate = deg2rad(20);
minTravRate = -deg2rad(20);

% Travel rate controller
KpTravRate = 2.45;
KiTravRate = 0;
KdTravRate = 0;
maxIntegralTravRate = inf;
minIntegralTravRate = -inf;
maxTravForce = 0.9;
minTravForce = -0.9;

% Compensation coefficients
aGravityCompensation = 0.001;
bGravityCompensation = 0.001;
cGravityCompensation = 0.001;

% Tuning reference values
tuningPitchRef = deg2rad(0);
tuningElevRef = deg2rad(24.5);
tuningTravRef = deg2rad(45);
tuningPitchRateRef = maxPitchRate;
tuningElevRateRef = maxElevRate;
tuningTravRateRef = maxTravRate;