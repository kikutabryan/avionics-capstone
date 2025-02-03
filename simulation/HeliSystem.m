 %#ok<*PROPLC>
%{
Global coordinate system is defined as a NED coordinate system. Where the
x-axis is defined as the north direction, the y-axis is defined as the 
east direction, and the z-axis is defined as the down direction.

Units are in metric.
Force - [N]
Distance - [m]
Torque - [N*m]
%}

classdef HeliSystem < matlab.System
    properties (Nontunable)
        V_max = 30;
        a_thrust = 0.0053;
        b_thrust = 0.0022;
        a_torque = 0.0003;
        b_torque = 0.0004;

        b_1 = 0.20;
        b_2 = 0.238;
        b_3 = 25.75 * 0.0254;
        b_4 = 0.0513;

        L_h = 6.985 * 0.0254;
        L_a = 25.75 * 0.0254;
        L_w = 18.125 * 0.0254;

        g = 9.81;

        T_stat_pitch = 0.001;
        T_stat_elev = 0.001;
        T_stat_trav = 0.005;

        T_coeff_pitch = 0.1;
        T_coeff_elev = 0.1;
        T_coeff_trav = 0.1;

        m_f = 0.721;
        m_b = 0.721;
        m_w = 1.914;

        ax_pitch = [-1; 0; 0];
        ax_elev = [0; 1; 0];
        ax_trav = [0; 0; -1];

        F_f = [0; 0; -1];
        F_b = [0; 0; -1];

        T_f = [0; 0; 1];
        T_b = [0; 0; -1];

        pitch_init = deg2rad(0);
        elev_init = deg2rad(0);
        trav_init = deg2rad(0);

        pitch_max = deg2rad(90);
        pitch_min = deg2rad(-90);

        elev_max = deg2rad(30);
        elev_min = deg2rad(-30);

        trav_max = deg2rad(180);
        trav_min = deg2rad(-180);

        theta = deg2rad(17.1);
    end

    properties (Access = protected)
        ddot_p = 0;
        ddot_e = 0;
        ddot_t = 0;

        dot_p = 0;
        dot_e = 0;
        dot_t = 0;

        prev_time = 0;

        pitch
        elev
        trav

        J_pitch

        r_w_init
        r_b_init
        r_f_init
        r_offset

        W_f
        W_b
        W_w

        T_dyn_pitch
        T_dyn_elev
        T_dyn_trav
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Initial angles
            obj.pitch = obj.pitch_init;
            obj.elev = obj.elev_init;
            obj.trav = obj.trav_init;

            % Friction
            obj.T_dyn_pitch = obj.T_stat_pitch / 5;
            obj.T_dyn_elev = obj.T_stat_elev / 5;
            obj.T_dyn_trav = obj.T_stat_trav / 5;

            % Moments of inertia
            obj.J_pitch = obj.m_f * obj.L_h^2 + obj.m_b + obj.L_h^2;  % [kg*m^2]

            % Points
            obj.r_w_init = [
                -obj.b_2 - obj.b_1 * cos(obj.theta);
                0;
                -obj.b_1 * sin(obj.theta) + obj.b_4
            ];  % Position of the weight
            obj.r_b_init = [
                obj.b_3;
                -obj.L_h;
                obj.b_4;
            ];  % Position of the back motor
            obj.r_f_init = [
                obj.b_3;
                obj.L_h;
                obj.b_4;
            ];  % Position of the front motor
            obj.r_offset = [
                0;
                0;
                obj.b_4;
            ];

            % Weights
            obj.W_f = [
                0;
                0;
                obj.m_f * obj.g;
            ];  % Weight of the front motor
            obj.W_b = [
                0;
                0;
                obj.m_b * obj.g;
            ];  % Weight of the back motor
            obj.W_w = [
                0;
                0;
                obj.m_w * obj.g;
            ];  % Weight of the counterweight
        end
        
        function [pitch, elev, trav] = stepImpl(obj, V_f, V_b)
            % Clamp voltages
            V_f_clamp = obj.clamp_motor_voltage(V_f);
            V_b_clamp = obj.clamp_motor_voltage(V_b);

            % Compute forces and torques magnitudes from voltages
            F_f_mag = obj.compute_motor_force(V_f_clamp);
            F_b_mag = obj.compute_motor_force(V_b_clamp);
            T_f_mag = obj.compute_motor_torque(V_f_clamp);
            T_b_mag = obj.compute_motor_torque(V_b_clamp);

            % Create map of point masses
            masses.front = obj.m_f;
            masses.back = obj.m_b;
            masses.weight = obj.m_w;

            % Create map of motor forces
            forces.front = obj.F_f * F_f_mag;
            forces.back = obj.F_b * F_b_mag;

            % Create map of motor torques
            torques.front = obj.T_f * T_f_mag;
            torques.back = obj.T_b * T_b_mag;

            % Create map of angles
            angles.pitch = obj.pitch;
            angles.elev = obj.elev;
            angles.trav = obj.trav;

            % Compute rotated points
            points.front = HeliSystem.rotation_heli_point(obj.r_f_init, obj.r_offset, angles);
            points.back = HeliSystem.rotation_heli_point(obj.r_b_init, obj.r_offset, angles);
            points.weight = HeliSystem.rotation_body_point(obj.r_w_init, angles);

            % Compute rotated axes of rotation
            % ax_pitch_rot = obj.rotation_pitch(obj.ax_pitch, angles);
            ax_elev_rot = obj.rotation_elev(obj.ax_elev, angles);
            ax_trav_rot = obj.ax_trav;

            % Compute net torque
            T_net = obj.torque_net(forces, torques, angles, points);

            % Resolve torque about rotation axis
            T_pitch = obj.L_h * (F_f_mag - F_b_mag);
            T_elev = dot(T_net, ax_elev_rot);
            T_trav = dot(T_net, ax_trav_rot);

            % Determine friction torque about each axis
            T_pitch = HeliSystem.add_bearing_friction(T_pitch, obj.dot_p, obj.T_stat_pitch, obj.T_dyn_pitch, obj.T_coeff_pitch);
            T_elev = HeliSystem.add_bearing_friction(T_elev, obj.dot_e, obj.T_stat_elev, obj.T_dyn_elev, obj.T_coeff_elev);
            T_trav = HeliSystem.add_bearing_friction(T_trav, obj.dot_t, obj.T_stat_trav, obj.T_dyn_trav, obj.T_coeff_trav);

            % Create torque vector
            T = [
                T_pitch;
                T_elev;
                T_trav;
            ];

            % Compute moment of inertia vector (wrt each heli axis)
            J_elev = HeliSystem.moment_of_inertia(masses, points, ax_elev_rot);
            J_trav = HeliSystem.moment_of_inertia(masses, points, ax_trav_rot);
            J = [
                obj.J_pitch;
                J_elev;
                J_trav
            ];

            % Compute angular accelerations
            alphas = T ./ J;

            % Get individual accelerations
            ddot_p = alphas(1);
            ddot_e = alphas(2);
            ddot_t = alphas(3);

            % Get the time
            curr_time = getCurrentTime(obj);
            delta_time = curr_time - obj.prev_time;

            % Update velocities
            dot_p = obj.dot_p + (ddot_p + obj.ddot_p) / 2 * delta_time;
            dot_e = obj.dot_e + (ddot_e + obj.ddot_e) / 2 * delta_time;
            dot_t = obj.dot_t + (ddot_t + obj.ddot_t) / 2 * delta_time;

            % Update positions
            pitch = obj.pitch + (dot_p + obj.dot_p) / 2 * delta_time;
            elev = obj.elev + (dot_e + obj.dot_e) / 2 * delta_time;
            trav = obj.trav + (dot_t + obj.dot_t) / 2 * delta_time;

            % Enforce bounds
            [pitch, dot_p, ddot_p] = HeliSystem.enforce_bounds(pitch, dot_p, ddot_p, obj.pitch_max, obj.pitch_min);
            [elev, dot_e, ddot_e] = HeliSystem.enforce_bounds(elev, dot_e, ddot_e, obj.elev_max, obj.elev_min);
            [trav, dot_t, ddot_t] = HeliSystem.enforce_bounds(trav, dot_t, ddot_t, obj.trav_max, obj.trav_min);

            % Set attributes
            obj.ddot_p = ddot_p;
            obj.ddot_e = ddot_e;
            obj.ddot_t = ddot_t;
            
            obj.dot_p = dot_p;
            obj.dot_e = dot_e;
            obj.dot_t = dot_t;

            obj.pitch = pitch;
            obj.elev = elev;
            obj.trav = trav;
            
            obj.prev_time = curr_time;
        end

        function T_net = torque_net(obj, forces, torques, angles, points)
            % Unpackages forces
            F_f = forces.front;
            F_b = forces.back;

            % Unpackages torques
            T_f = torques.front;
            T_b = torques.back;

            % Unpackages points
            r_f_rot = points.front;
            r_b_rot = points.back;
            r_w_rot = points.weight;

            % Compute rotated motor force vectors
            F_f_rot = HeliSystem.rotation_thrust(F_f, angles);
            F_b_rot = HeliSystem.rotation_thrust(F_b, angles);

            % Compute rotated motor torque vectors
            T_f_rot = HeliSystem.rotation_torque(T_f, angles);
            T_b_rot = HeliSystem.rotation_torque(T_b, angles);

            % Torque from motors
            T_motors = T_f_rot + T_b_rot + cross(r_f_rot, F_f_rot) + cross(r_b_rot, F_b_rot);

            % Torque from weights
            T_weights = cross(r_w_rot, obj.W_w) + cross(r_f_rot, obj.W_f) + cross(r_b_rot, obj.W_b);

            % Compute the net torque
            T_net = T_motors + T_weights;
        end

        function V_clamp = clamp_motor_voltage(obj, V)
            if V > obj.V_max
                V_clamp = obj.V_max;
            elseif V < -obj.V_max
                V_clamp = -obj.V_max;
            else
                V_clamp = V;
            end
        end

        function F = compute_motor_force(obj, V)
            F = obj.a_thrust * abs(V)^2 + obj.b_thrust * abs(V);

            % Flip force for negative voltage
            if V < 0
                F = -F;
            end
        end

        function T = compute_motor_torque(obj, V)
            T = obj.a_torque * abs(V)^2 + obj.b_torque * abs(V);

            % Flip torque for negative voltage
            if V < 0
                T = -T;
            end
        end
    end

    methods (Static)
        function [a_bound, dot_a_bound, ddot_a_bound] = enforce_bounds(a, dot_a, ddot_a, upper, lower)
            if a > upper
                a_bound = upper;
                dot_a_bound = 0;
                ddot_a_bound = 0;
            elseif a < lower
                a_bound = lower;
                dot_a_bound = 0;
                ddot_a_bound = 0;
            else
                a_bound = a;
                dot_a_bound = dot_a;
                ddot_a_bound = ddot_a;
            end
        end

        function T_new = add_bearing_friction(T, omega, T_stat, T_dyn, T_coeff)
            if omega == 0 && abs(T) < T_stat
                % Static friction
                T_new = 0;
            else
                % Dynamic friction
                T_new = T - (T_coeff * omega + T_dyn);
            end
        end

        function R = rotation_matrix(axis, theta)
            switch axis
                case 1
                    R = [
                        1, 0, 0;
                        0, cos(theta), -sin(theta);
                        0, sin(theta), cos(theta);
                    ];
                case 2
                    R = [
                        cos(theta), 0, sin(theta);
                        0, 1, 0;
                        -sin(theta), 0, cos(theta);
                    ];
                case 3
                    R = [
                        cos(theta), -sin(theta), 0;
                        sin(theta), cos(theta), 0;
                        0, 0, 1;
                    ];
            end
        end

        function r_rot = rotation_heli_point(r, offset, angles)           
            pitch = angles.pitch;
            elev = angles.elev;
            trav = angles.trav;
            R_x = HeliSystem.rotation_matrix(1, -pitch);
            R_y = HeliSystem.rotation_matrix(2, elev);
            R_z = HeliSystem.rotation_matrix(3, -trav);
            r_rot = R_z * R_y * (R_x * (r - offset) + offset);
        end

        function r_rot = rotation_body_point(r, angles)
            elev = angles.elev;
            trav = angles.trav;
            R_y = HeliSystem.rotation_matrix(2, elev);
            R_z = HeliSystem.rotation_matrix(3, -trav);
            r_rot = R_z * R_y * r;
        end

        function F_rot = rotation_thrust(F, angles)
            pitch = angles.pitch;
            elev = angles.elev;
            trav = angles.trav;
            R_x = HeliSystem.rotation_matrix(1, -pitch);
            R_y = HeliSystem.rotation_matrix(2, elev);
            R_z = HeliSystem.rotation_matrix(3, -trav);
            F_rot = R_z * R_y * R_x * F;
        end

        function T_rot = rotation_torque(T, angles)
            pitch = angles.pitch;
            elev = angles.elev;
            trav = angles.trav;
            R_x = HeliSystem.rotation_matrix(1, -pitch);
            R_y = HeliSystem.rotation_matrix(2, elev);
            R_z = HeliSystem.rotation_matrix(3, -trav);
            T_rot = R_z * R_y * R_x * T;
        end

        function ax_pitch_rot = rotation_pitch(ax_pitch, angles)
            elev = angles.elev;
            trav = angles.trav;
            R_y = HeliSystem.rotation_matrix(2, elev);
            R_z = HeliSystem.rotation_matrix(3, -trav);
            ax_pitch_rot = R_z * R_y * ax_pitch;
        end

        function ax_elev_rot = rotation_elev(ax_elev, angles)
            trav = angles.trav;
            R_z = HeliSystem.rotation_matrix(3, -trav);
            ax_elev_rot = R_z * ax_elev;
        end

        function J = moment_of_inertia(masses, points, axis)
            % Normalize axis vector
            axis = axis / norm(axis);

            % Compute moment of inertia of point masses
            J = 0;

            % Get field names from the struct
            mass_fields = fieldnames(masses);

            for i = 1:length(masses)
                field = mass_fields{i};  % Extract the field name (e.g., 'front', 'back', 'weight')

                % Extract mass and corresponding point
                mass = masses.(field);
                r = points.(field);

                % Compute the perpendicular component of the position vector
                r_mag2 = dot(r, r);
                r_proj2 = dot(r, axis)^2;

                % Add point mass contribution to the moment of inertia
                J = J + mass * (r_mag2 - r_proj2);
            end
        end
    end
end