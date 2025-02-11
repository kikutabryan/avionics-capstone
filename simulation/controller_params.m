%% Pitch Rate Controller
Kp_dotp = 3.8;
Ki_dotp = 1.2;
Kd_dotp = 0;
% Range of delta F [N]
max_dotp = 3;
min_dotp = -3;

%% Pitch Angle Controller
Kp_p = 1.5;
Ki_p = 0;
Kd_p = 1;
% Range of pitch rate [deg/s]
max_p = deg2rad(45);
min_p = -deg2rad(45);

%% Travel Rate Controller
Kp_dott = 0.3;
Ki_dott = 0.05;
Kd_dott = 0.2;
% Range of pitch [deg]
max_dott = deg2rad(15);
min_dott = -deg2rad(15);

%% Travel Angle Controller
Kp_t = 0.4;
Ki_t = 0.0;
Kd_t = 0.16;
% Range of travel rate [deg/s]
max_t = deg2rad(45);
min_t = -deg2rad(45);

%% Elevation Rate Controller
Kp_dote = 5;
Ki_dote = 10;
Kd_dote = 0;
% Range of delta F [N]
max_dote = 4;
min_dote = 0;
max_int = inf;
min_int = 0;

%% Elevation Angle Controller
Kp_e = 1.5;
Ki_e = 0;
Kd_e = 0;
% Range of elevation rate [deg/s]
max_e = deg2rad(45);
min_e = -deg2rad(45);