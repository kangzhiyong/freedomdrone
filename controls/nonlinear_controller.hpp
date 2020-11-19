#pragma once

#include "free_point.hpp"

#define DRONE_MASS_KG 0.5
#define GRAVITY  -9.81
#define MOI {0.005, 0.005, 0.01}
#define MAX_THRUST 10.0
#define MAX_TORQUE 1


class NonlinearController
{
private:
    float Kp_pos{ 6 }; //12.0,
    float Kp_vel{ 4 }; //8.0,
    float Kp_alt{ 4.0 };
    float Kp_hdot{ 1.5 }; //2.0,

    float Kp_roll{ 8 }; //8, 6.5,
    float Kp_pitch{ 8 }; //8, #6.5,
    float Kp_yaw{ 4.5 }; //4.5, #4.5,

    float Kp_p{ 20 }; //10,
    float Kp_q{ 20 }; //10,
    float Kp_r{ 5 }; //10,

    float max_tilt{ 1.0 };
    float max_ascent_rate{ 5 };
    float max_descent_rate{ 2 };
    float max_speed{ 5.0 };
public:
    NonlinearController() {}

    point3D trajectory_control(vector<point3D> position_trajectory, vector<float> yaw_trajectory, vector<float> time_trajectory,
        float current_time, point3D& position_cmd, point3D& velocity_cmd, float& yaw_cmd)
    {
        /*Generate a commanded position, velocity and yaw based on the
        trajectory

        Args :
            position_trajectory: list of 3 - element numpy arrays, NED positions
            yaw_trajectory : list yaw commands in radians
            time_trajectory : list of times(in seconds) that correspond to the
            position and yaw commands
            current_time : float corresponding to the current time in seconds

            Returns : tuple(commanded position, commanded velocity, commanded yaw)
        */
        vector<float> rel_times = time_trajectory;
        for (int i = 0; i < time_trajectory.size(); i++) {
            rel_times[i] = abs(time_trajectory[i] - current_time);
        }
        int ind_min = min_element(rel_times.begin(), rel_times.end()) - rel_times.begin();

        float time_ref = time_trajectory[ind_min];
        point3D position0, position1;
        float time0, time1;
        if (current_time < time_ref)
        {
            position0 = position_trajectory[ind_min - 1];
            position1 = position_trajectory[ind_min];

            time0 = time_trajectory[ind_min - 1];
            time1 = time_trajectory[ind_min];
            yaw_cmd = yaw_trajectory[ind_min - 1];
        }
        else
        {
            yaw_cmd = yaw_trajectory[ind_min];
            if (ind_min >= (position_trajectory.size() - 1))
            {
                position0 = position_trajectory[ind_min]
                    position1 = position_trajectory[ind_min]

                    time0 = 0.0
                    time1 = 1.0
            }
            else
            {
                position0 = position_trajectory[ind_min];
                position1 = position_trajectory[ind_min + 1];
                time0 = time_trajectory[ind_min];
                time1 = time_trajectory[ind_min + 1];
            }
        }
        point3D tmp = position1 - position0;
        position_cmd = tmp * (current_time - time0) / (time1 - time0) + position0;
        velocity_cmd = tmp / (time1 - time0);
    }

    point2D lateral_position_control(point2D local_position_cmd, point2D local_velocity_cmd, point2D local_position, point2D local_velocity, point2D acceleration_ff)
    {
        /*Generate horizontal acceleration commands for the vehicle in the
        local frame

        Args :
            local_position_cmd: desired 2D position in local frame
            [north, east]
            local_velocity_cmd : desired 2D velocity in local frame
            [north_velocity, east_velocity]
            local_position : vehicle position in the local frame
            [north, east]
            local_velocity : vehicle velocity in the local frame
            [north_velocity, east_velocity]
            acceleration_cmd : feedforward acceleration command

            Returns : desired vehicle 2D acceleration in the local frame
            [north, east]
        */
        point2D velocity_cmd = Kp_pos * (local_position_cmd - local_position);

        //Limit speed
        float velocity_norm = np.sqrt(velocity_cmd[0] * velocity_cmd[0] + velocity_cmd[1] * velocity_cmd[1]);

        if (velocity_norm > max_speed)
        {
            velocity_cmd = velocity_cmd * max_speed / velocity_norm;
        }
        point2D acceleration_cmd = acceleration_ff + Kp_pos * (local_position_cmd - local_position) + Kp_vel * (local_velocity_cmd - local_velocity);

        return acceleration_cmd;
    }

    float altitude_control(float altitude_cmd, float vertical_velocity_cmd, float altitude, float vertical_velocity, point3D attitude, float acceleration_ff = 0.0)
    {
        /*Generate vertical acceleration (thrust) command

        Args :
            altitude_cmd : desired vertical position(+up)
            vertical_velocity_cmd : desired vertical velocity(+up)
            altitude : vehicle vertical position(+up)
            vertical_velocity : vehicle vertical velocity(+up)
            acceleration_ff : feedforward acceleration command(+up)

            Returns : thrust command for the vehicle(+up)
        */

        float hdot_cmd = Kp_alt * (altitude_cmd - altitude) + vertical_velocity_cmd;

        // Limit the ascent / descent rate
        float hdot_cmd = np.clip(hdot_cmd, -max_descent_rate, max_ascent_rate);

        float acceleration_cmd = acceleration_ff + Kp_hdot * (hdot_cmd - vertical_velocity);

        float R33 = cos(attitude[0]) * cos(attitude[1]);
        float thrust = DRONE_MASS_KG * acceleration_cmd / R33;

        if (thrust > MAX_THRUST)
        {
            thrust = MAX_THRUST;
        }
        else if (thrust < 0.0)
        {
            thrust = 0.0;
        }
        return thrust;
    }

    point2D roll_pitch_controller(point2D acceleration_cmd, point3D attitude, float thrust_cmd)
    {
        /* Generate the rollrate and pitchrate commands in the body frame

        Args :
            target_acceleration : 2 - element numpy array
            (north_acceleration_cmd, east_acceleration_cmd) in m / s ^ 2
            attitude : 3 - element numpy array(roll, pitch, yaw) in radians
            thrust_cmd : vehicle thruts command in Newton

            Returns : 2 - element numpy array, desired rollrate(p) and
            pitchrate(q) commands in radians / s
        */
        //Calculate rotation matrix
        Mat3x3F R = euler2RM(attitude[0], attitude[1], attitude[2]);
        float c_d = thrust_cmd / DRONE_MASS_KG;
        float p_cmd, q_cmd;
        if (thrust_cmd > 0.0)
        {
            float target_R13 = -clip(acceleration_cmd[0].item() / c_d, -max_tilt, max_tilt); // - min(max(acceleration_cmd[0].item() / c_d, -max_tilt), max_tilt)
            float target_R23 = -clip(acceleration_cmd[1].item() / c_d, -max_tilt, max_tilt); // - min(max(acceleration_cmd[1].item() / c_d, -max_tilt), max_tilt)

            p_cmd = (1 / R[2, 2]) * (-R[1, 0] * Kp_roll * (R[0, 2] - target_R13) + R[0, 0] * Kp_pitch * (R[1, 2] - target_R23));
            q_cmd = (1 / R[2, 2]) * (-R[1, 1] * Kp_roll * (R[0, 2] - target_R13) + R[0, 1] * Kp_pitch * (R[1, 2] - target_R23));
        }
        else  // Otherwise command no rate
        {
            cout << "negative thrust command") << endl;
            p_cmd = 0.0;
            q_cmd = 0.0;
            thrust_cmd = 0.0;
        }
        return { p_cmd, q_cmd };
    }

    point3D body_rate_control(point3D body_rate_cmd, point3D body_rate)
    {
        /* Generate the roll, pitch, yaw moment commands in the body frame

        Args :
            body_rate_cmd : 3 - element numpy array(p_cmd, q_cmd, r_cmd)
            in radians / second ^ 2
            attitude : 3 - element numpy array(p, q, r) in radians / second ^ 2

            Returns : 3 - element numpy array, desired roll moment, pitch moment, and
            yaw moment commands in Newtons* meters
        */
        point3D Kp_rate({ Kp_p, Kp_q, Kp_r });
        point3D rate_error = body_rate_cmd - body_rate;

        point3D moment_cmd = MOI * (Kp_rate * rate_error);
        if (norm(moment_cmd) > MAX_TORQUE)
        {
            moment_cmd = moment_cmd * MAX_TORQUE / norm(moment_cmd);
        }
        return moment_cmd;
    }

    float yaw_control(float yaw_cmd, float yaw)
    {
        /* Generate the target yawrate

        Args :
            yaw_cmd : desired vehicle yaw in radians
            yaw : vehicle yaw in radians

            Returns : target yawrate in radians / sec
        */

        // Ensure the target is within range of 0 to 2 * pi
        yaw_cmd = np.mod(yaw_cmd, 2.0 * np.pi);

        float yaw_error = yaw_cmd - yaw;
        if (yaw_error > M_PI)
        {
            yaw_error = yaw_error - 2.0 * M_PI;
        }
        else if (yaw_error < -M_PI)
        {
            yaw_error = yaw_error + 2.0 * M_PI;
        }

        float yawrate_cmd = Kp_yaw * yaw_error;
        return yawrate_cmd;
    }
};

