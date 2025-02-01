 %#ok<*PROPLC>

classdef HeliSystem < matlab.System
    properties (Nontunable)
        % Motor forces and torques
        V_max = 30;
        a_thrust = 0.0053;
        b_thrust = 0.0022;
        a_torque = 0.0003;
        b_torque = 0.0004;

        % Bar lengths
        b_1 = 0.1;  % [m]
        b_2 = 0.2;  % [m]
        b_3 = 0.3;  % [m]
        b_4 = 0.1;  % [m]

        % Provided lengths
        L_h = 6.985 * 0.0254;  % [m]
        L_a = 25.75 * 0.0254;  % [m]
        L_w = 18.125 * 0.0254;  % [m]

        % Angles
        theta = 0.1;  % [rad]

        % Gravitational constant
        g = 9.81;  % [m/s^2]

        % Masses
        m_f = 0.713;  % [kg]
        m_b = 0.713;  % [kg]
        m_w = 1.914;  % [kg]

        % Moments of inertia
        J_pitch;

        % Points
        r_w_init;
        r_b_init;
        r_f_init;
        r_offset;

        % Axis
        ax_pitch = [1; 0; 0];  % Travel axis
        ax_elev = [0; 1; 0];  % Elevation axis
        ax_trav = [0; 0; 1];  % Travel axis

        % Weights
        W_f;
        W_b;
        W_w;

        % Thrusts
        F_f = [0; 0; -1];  % Unit vector of thrust
        F_b = [0; 0; -1];  % Unit vector of thrust

        % Torques (f and b are inverted since they spin in different directions)
        T_f = [0; 0; 1];  % Unit vector of torque
        T_b = [0; 0; -1];  % Unit vector of torque
    end

    methods (Access = public)
        function accels = compute_angular_accel(obj, V_f, V_b, pitch, elev, trav)
            % Clamp voltages
            V_f = obj.clamp_motor_voltage(V_f);
            V_b = obj.clamp_motor_voltage(V_b);

            % Compute forces and torques magnitudes from voltages
            F_f_mag = obj.compute_motor_force(V_f);
            F_b_mag = obj.compute_motor_force(V_b);
            T_f_mag = obj.compute_motor_torque(V_f);
            T_b_mag = obj.compute_motor_torque(V_b);

            % Create map of point masses
            masses_keys = {"weight", "front", "back"};
            masses_vals = [obj.m_f, obj.m_f, obj.m_b];
            masses = containers.Map(masses_keys, masses_vals);

            % Create map of motor forces
            forces_keys = {"front", "back"};
            forces_vals = [obj.F_f * F_f_mag, obj.F_b * F_b_mag];
            forces = containers.Map(forces_keys, forces_vals);

            % Create map of motor torques
            torques_keys = {"front", "back"};
            torques_vals = [obj.T_f * T_f_mag, obj.T_b * T_b_mag];
            torques = containers.Map(torques_keys, torques_vals);

            % Create map of angles
            angles_keys = {"pitch", "elev", "trav"};
            angles_vals = [pitch, elev, trav];
            angles = containers.Map(angles_keys, angles_vals);

            % Compute rotated points
            points_keys = {"weight", "front", "back"};
            r_w_rot = PlantObject.rotation_body_point(obj.r_w_init, angles);
            r_f_rot = PlantObject.rotation_heli_point(obj.r_f_init, obj.r_offset, angles);
            r_b_rot = PlantObject.rotation_heli_point(obj.r_b_init, obj.r_offset, angles);
            points_vals = [r_w_rot, r_f_rot, r_b_rot];
            points = containers.Map(points_keys, points_vals);

            % Compute rotated axes of rotation
            ax_pitch_rot = obj.rotation_pitch(obj.ax_pitch, angles);
            ax_elev_rot = obj.rotation_trav(obj.ax_trav, angles);
            ax_trav_rot = obj.rotation_elev(obj.ax_elev, angles);

            % Compute net torque
            T_net = torque_net(forces, torques, angles, points);

            % Resolve torque about rotation axis
            T_pitch = dot(T_net, ax_pitch_rot);
            T_trav = dot(T_net, ax_elev_rot);
            T_elev = dot(T_net, ax_trav_rot);
            T = [
                T_pitch;
                T_elev;
                T_trav;
            ];

            % Compute moment of inertia vector (wrt each heli axis)
            J_elev = PlantObject.moment_of_inertia(masses, points, ax_elev_rot);
            J_trav = PlantObject.moment_of_inertia(masses, points, ax_trav_rot);
            J = [
                obj.J_pitch;
                J_elev;
                J_trav
            ];

            % Compute angular accelerations
            accels = T ./ J;
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.J_pitch = obj.m_f * obj.L_h^2 + obj.m_b + obj.L_h^2;  % [kg*m^2]

            % Points
            obj.r_w_init = [
                -obj.b_2 - obj.b_1 * cos(obj.theta);
                0;
                obj.b_1 * sin(obj.theta)
            ];  % Position of the weight
            obj.r_b_init = [
                obj.b_3;
                -obj.L_h;
                -obj.b_4;
            ];  % Position of the back motor
            obj.r_f_init = [
                obj.b_3;
                obj.L_h;
                -obj.b_4;
            ];  % Position of the front motor
            obj.r_offset = [
                0;
                0;
                -obj.b4;
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

        function T_net = torque_net(obj, forces, torques, angles, points)
            % Unpackages forces
            F_f = forces("front");
            F_b = forces("back");

            % Unpackages torques
            T_f = torques("front");
            T_b = torques("back");

            % Unpackages angles
            pitch = angles("pitch");
            elev = angles("elev");
            trav = angles("trav");

            % Unpackages points
            r_w_rot = points("weight");
            r_f_rot = points("front");
            r_b_rot = points("back");

            % Compute rotated motor force vectors
            F_f_rot = PlantObject.rotation_thrust(F_f, pitch, elev, trav);
            F_b_rot = PlantObject.rotation_thrust(F_b, pitch, elev, trav);

            % Compute rotated motor torque vectors
            T_f_rot = PlantObject.rotation_torque(T_f, pitch, elev, trav);
            T_b_rot = PlantObject.rotation_torque(T_b, pitch, elev, trav);

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
                V_clamp = -obj.Vmax;
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
        function R = rotation_matrix(axis, theta)
            switch axis
                case 'x'
                    R = [
                        1, 0, 0;
                        0, cos(theta), -sin(theta);
                        0, sin(theta), cos(theta);
                    ];
                case 'y'
                    R = [
                        cos(theta), 0, sin(theta);
                        0, 1, 0;
                        -sin(theta), 0, cos(theta);
                    ];
                case 'z'
                    R = [
                        cos(theta), -sin(theta), 0;
                        sin(theta), cos(theta), 0;
                        0, 0, 1;
                    ];
            end
        end

        function r_rot = rotation_heli_point(r, offset, angles)
            pitch = angles("pitch");
            elev = angles("elev");
            trav = angles("trav");
            R_x = rotation_matrix('x', -pitch);
            R_y = rotation_matrix('y', elev);
            R_z = rotation_matrix('z', -trav);
            r_rot = R_z * R_y * (R_x * (r - offset) + offset);
        end

        function r_rot = rotation_body_point(r, angles)
            elev = angles("elev");
            trav = angles("trav");
            R_y = rotation_matrix('y', elev);
            R_z = rotation_matrix('z', -trav);
            r_rot = R_z * R_y * r;
        end

        function F_rot = rotation_thrust(F, angles)
            pitch = angles("pitch");
            elev = angles("elev");
            trav = angles("trav");
            R_x = rotation_matrix('x', -pitch);
            R_y = rotation_matrix('y', elev);
            R_z = rotation_matrix('z', -trav);
            F_rot = R_z * R_y * R_x * F;
        end

        function T_rot = rotation_torque(T, angles)
            pitch = angles("pitch");
            elev = angles("elev");
            trav = angles("trav");
            R_x = rotation_matrix('x', -pitch);
            R_y = rotation_matrix('y', elev);
            R_z = rotation_matrix('z', -trav);
            T_rot = R_z * R_y * R_x * T;
        end

        function ax_pitch_rot = rotation_pitch(ax_pitch, angles)
            elev = angles("elev");
            trav = angles("trav");
            R_y = PlantObject.rotation_matrix('y', elev);
            R_z = PlantObject.rotation_matrix('z', -trav);
            ax_pitch_rot = R_z * R_y * ax_pitch;
        end

        function ax_elev_rot = rotation_elev(ax_elev, angles)
            trav = angles("trav");
            R_z = PlantObject.rotation_matrix('z', -trav);
            ax_elev_rot = R_z * ax_elev;
        end
    
        function ax_trav_rot = rotation_trav(ax_trav)
            ax_trav_rot = -ax_trav;
        end

        function J = moment_of_inertia(masses, points, axis)
            % Normalize axis vector
            axis = axis / norm(axis);

            % Compute moment of inertia of point masses
            J = 0;
            for i = 1:length(masses)
                % Compute the difference between the magnitude of the position
                % vector and the magnitude of the position vector projected onto
                % the axis vector to get the magnitude of the perp component to the axis
                r = points(i, :);
                r_mag2 = dot(r, r);
                r_proj2 = dot(r, axis)^2;

                % Add point mass to the moment of inertia
                J = J + masses(i) * (r_mag2 - r_proj2);
            end
        end
    end
end