%% Initial Angle Positions
pitch_0 = deg2rad(0);
elev_0 = deg2rad(-30);
trav_0 = deg2rad(0);

%% Initial Angular Velocities
dot_pitch_0 = deg2rad(0);
dot_elev_0 = deg2rad(0);
dot_trav_0 = deg2rad(0);

%% Initial Angular Accelerations
ddot_pitch_0 = deg2rad(0);
ddot_elev_0 = deg2rad(0);
ddot_trav_0 = deg2rad(0);

%% Limits
pitch_min = deg2rad(-90);
pitch_max = deg2rad(90);

elev_min = deg2rad(-30);
elev_max = deg2rad(30);

trav_min = deg2rad(-180);
trav_max = deg2rad(180);