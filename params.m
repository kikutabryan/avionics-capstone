% Constants
g = 9.805;  % [m/s^2]

% Masses
M_f = 0.713;  % [kg]
M_b = M_f;  % [kg]
M_w = 1.914;  % [kg]

% Distances
L_h = 6.985 * 0.0254;  % [m]
L_a = 25.75 * 0.0254;  % [m]
L_w = 18.125 * 0.0254;  % [m]

% Forces
F_steady = 3.65;  % [N] - Force required to maintain steady level flight

% Moments of inertia
J_p = (M_b * L_h^2) + (M_f * L_h^2);  % [kg*m^2]
J_t = (M_w * L_w^2) + ((M_b + M_f) * (L_h^2 + L_a^2));  % [kg*m^2]

% Transfer functions
dotP_deltaF_num = L_h;
dotP_deltaF_den = [J_p, 0];

ddotP_deltaF_num = L_h;
ddotP_deltaF_den = J_p;

dotLambda_P_num = L_a * F_steady;
dotLambda_P_den = [J_t, 0];

ddotLambda_P_num = L_a * F_steady;
ddotLambda_P_den = J_t;

% PID limits
deltaF_max = 3;  % [N]