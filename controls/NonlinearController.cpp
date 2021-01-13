#include "NonlinearController.hpp"
#include "DroneUtils.hpp"

V3F MOI = {0.005, 0.005, 0.01};

V3F NonlinearController::trajectory_control(vector<V3F> position_trajectory, vector<float> yaw_trajectory, vector<float> time_trajectory,
        float current_time, V3F& position_cmd, V3F& velocity_cmd, float& yaw_cmd)
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
    V3F position0, position1;
    float time0, time1;
    float time_ref = time_trajectory[ind_min];
    if (ind_min == 0)
    {
        position0 = position_trajectory[ind_min];
        position1 = position_trajectory[ind_min];
        time0 = 0.0;
        time1 = 1.0;
        yaw_cmd = yaw_trajectory[ind_min];
    }
    else if (current_time < time_ref)
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
            position0 = position_trajectory[ind_min];
            position1 = position_trajectory[ind_min];

            time0 = 0.0;
            time1 = 1.0;
        }
        else
        {
            position0 = position_trajectory[ind_min];
            position1 = position_trajectory[ind_min + 1];
            time0 = time_trajectory[ind_min];
            time1 = time_trajectory[ind_min + 1];
        }
    }
    position_cmd = (position1 - position0) * (current_time - time0) / (time1 - time0) + position0;
    velocity_cmd = (position1 - position0) / (time1 - time0);
    return position_cmd;
}

V3F NonlinearController::lateral_position_control(V3F local_position_cmd, V3F local_velocity_cmd, V3F local_position, V3F local_velocity, V3F acceleration_ff)
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
    V3F acceleration_cmd = acceleration_ff;
    V3F velocity_cmd =  Kp_pos * (local_position_cmd - local_position);

    //Limit speed
    float velocity_norm = sqrt(velocity_cmd[0] * velocity_cmd[0] + velocity_cmd[1] * velocity_cmd[1]);
    if (velocity_norm > max_speed)
    {
        velocity_cmd = velocity_cmd * max_speed / velocity_norm;
    }
    acceleration_cmd = acceleration_ff + velocity_cmd + Kp_vel * (local_velocity_cmd - local_velocity);
    float acc_norm = sqrt(acceleration_cmd[0] * acceleration_cmd[0] + acceleration_cmd[1] * acceleration_cmd[1]);
    if (acc_norm > max_accel)
    {
        acceleration_cmd = acceleration_cmd * max_accel / acc_norm;
    }
    return acceleration_cmd;
}

float NonlinearController::altitude_control(float altitude_cmd, float vertical_velocity_cmd, float altitude, float vertical_velocity, SLR::Quaternion<float> attitude, float acceleration_ff)
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
    Mat3x3F R = attitude.RotationMatrix_IwrtB();

    float hdot_cmd = Kp_alt * (altitude_cmd - altitude) + vertical_velocity_cmd;

    // Limit the ascent / descent rate
    hdot_cmd = clip(hdot_cmd, -max_descent_rate, max_ascent_rate);
    float acceleration_cmd = acceleration_ff + Kp_hdot * (hdot_cmd - vertical_velocity);

    float thrust = -DRONE_MASS_KG * acceleration_cmd / R(2, 2);
    thrust = clip(thrust, (float)-MAX_THRUST, (float)MAX_THRUST);
    return thrust;
}

V3F NonlinearController::roll_pitch_controller(V3F acceleration_cmd, SLR::Quaternion<float> attitude, float thrust_cmd)
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
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float c_d = thrust_cmd / DRONE_MASS_KG;
    V3F target_R = acceleration_cmd / c_d;
    target_R = -clip(target_R, -max_tilt, max_tilt);
    float b_x_c_dot = Kp_roll * (R(0, 2) - target_R[0]);
    float b_y_c_dot = Kp_pitch * (R(1, 2) - target_R[1]);
    V3F pqrCmd;
    pqrCmd[0] = (1 / R(2, 2)) * (-R(1, 0) * b_x_c_dot + R(0, 0) * b_y_c_dot);
    pqrCmd[1] = (1 / R(2, 2)) * (-R(1, 1) * b_x_c_dot + R(0, 1) * b_y_c_dot);
    pqrCmd[2] = 0;
    return pqrCmd;
}

V3F NonlinearController::body_rate_control(V3F body_rate_cmd, V3F body_rate)
{
    /* Generate the roll, pitch, yaw moment commands in the body frame

    Args :
        body_rate_cmd : 3 - element numpy array(p_cmd, q_cmd, r_cmd)
        in radians / second ^ 2
        attitude : 3 - element numpy array(p, q, r) in radians / second ^ 2

        Returns : 3 - element numpy array, desired roll moment, pitch moment, and
        yaw moment commands in Newtons* meters
    */
    V3F Kp_rate({ Kp_p, Kp_q, Kp_r });
    V3F moment_cmd = Kp_rate * (body_rate_cmd - body_rate) * MOI;
    if (moment_cmd.mag() > MAX_TORQUE)
    {
        moment_cmd = moment_cmd * MAX_TORQUE / moment_cmd.mag();
    }
    return moment_cmd;
}

float NonlinearController::yaw_control(float yaw_cmd, float yaw)
{
    /* Generate the target yawrate

    Args :
        yaw_cmd : desired vehicle yaw in radians
        yaw : vehicle yaw in radians

        Returns : target yawrate in radians / sec
    */

    // Ensure the target is within range of 0 to 2 * pi
    yaw_cmd = fmodf(yaw_cmd, (float)(2.0 * M_PI));

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

void NonlinearController::UpdateEstimates(V3F pos, V3F vel, SLR::Quaternion<float> attitude, V3F omega)
{
    estAtt = attitude;
    estOmega = omega;
    estPos = pos;
    estVel = vel;
} 
