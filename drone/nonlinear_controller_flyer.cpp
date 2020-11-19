#include "nonlinear_controller_flyer.hpp"
#include "free_utils.hpp"

ControlsFlyer::ControlsFlyer(MavlinkConnection* conn) : UnityDrone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)()) & ControlsFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)()) & ControlsFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)()) & ControlsFlyer::state_callback));
    register_callback(ATTITUDE, ((void (Drone::*)()) & ControlsFlyer::attitude_callback));
    register_callback(RAW_GYROSCOPE, ((void (Drone::*)()) & ControlsFlyer::gyro_callback));
}

void ControlsFlyer::position_controller()
{
    float yaw_cmd;
    controller.trajectory_control(position_trajectory, yaw_trajectory, time_trajectory, time(0), local_position_target, local_velocity_target, yaw_cmd);
    attitude_target = point3D({0.0, 0.0, yaw_cmd});
    point2D acceleration_cmd = controller.lateral_position_control(
                                                           point2D({local_position_target[0], local_position_target[1]}),
                                                           point2D({local_velocity_target[0], local_velocity_target[1]}),
                                                           point2D({local_position()[0], local_position()[1]}),
                                                           point2D({local_velocity()[0], local_velocity()[1]}),
                                                           point2D({0, 0}));
    local_acceleration_target = point3D({acceleration_cmd[0], acceleration_cmd[1], 0.0});
}

void ControlsFlyer::attitude_controller()
{
    thrust_cmd = controller.altitude_control(-local_position_target[2], -local_velocity_target[2], -local_position()[2], -local_velocity()[2], attitude(), 9.81);
    point2D roll_pitch_rate_cmd = controller.roll_pitch_controller(point2D({local_acceleration_target[0], local_acceleration_target[1]}), attitude(), thrust_cmd);
    float yawrate_cmd = controller.yaw_control(attitude_target[2], attitude()[2]);
    body_rate_target = point3D({roll_pitch_rate_cmd[0], roll_pitch_rate_cmd[1], yawrate_cmd});
}

void ControlsFlyer::bodyrate_controller()
{
    point3D moment_cmd = controller.body_rate_control(body_rate_target, gyro_raw());
    cmd_moment(moment_cmd[0], moment_cmd[1], moment_cmd[2], thrust_cmd);
}

void ControlsFlyer::attitude_callback()
{
        if (flight_state == WAYPOINT)
        {
            attitude_controller();
        }
}

void ControlsFlyer::gyro_callback()
{
        if (flight_state == WAYPOINT)
        {
            bodyrate_controller();
        }
}

void ControlsFlyer::local_position_callback()
{
    if (flight_state == TAKEOFF)
    {
        if (-1.0 * local_position()[2] > 0.95 * target_position[2])
        {
            float time_mult = 0.5;
            // all_waypoints = calculate_box();
            load_test_trajectory(time_mult, position_trajectory, time_trajectory, yaw_trajectory);
            for (int i = 0; i < position_trajectory.size(); i++) {
                all_waypoints.push(position_trajectory[i]);
            }
            waypoint_number = -1;
            waypoint_transition();
        }
    }
    else if (flight_state == WAYPOINT)
    {
        if (time(0) > time_trajectory[waypoint_number])
        {
            if (all_waypoints.size() > 0)
            {
                waypoint_transition();
            }
            else if (norm(local_velocity()) < 1.0)
            {
                landing_transition();
            }
        }
    }
}

void ControlsFlyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if (global_position()[2] - global_home()[2] < 0.1)
        {
            if (abs(local_position()[2]) < 0.01)
            {
                disarming_transition();
            }
        }
    }
    if (flight_state == WAYPOINT)
    {
        position_controller();
    }
}

void ControlsFlyer::state_callback()
{
    if (in_mission)
    {
        if (flight_state == MANUAL)
        {
            arming_transition();
        }
        else if (flight_state == ARMING && armed())
        {
            takeoff_transition();
        }
        else if (flight_state == DISARMING && !armed() && !guided())
        {
            manual_transition();
        }
    }
}

void ControlsFlyer::calculate_box()
{
    cout << "Setting Home" << endl;
    all_waypoints.push({10.0, 0.0, -3.0});
    all_waypoints.push({10.0, 10.0, -3.0});
    all_waypoints.push({0.0, 10.0, -3.0});
    all_waypoints.push({0.0, 0.0, -3.0});
}

void ControlsFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_as_current_position();
    flight_state = ARMING;
}

void ControlsFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = TAKEOFF_ALTITUDE;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = TAKEOFF;
}

void ControlsFlyer::waypoint_transition()
{
    cout << "waypoint transition" << endl;
    if (all_waypoints.size() <= 0)
    {
        return;
    }
    waypoint_number = waypoint_number + 1;
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    flight_state = WAYPOINT;
}

void ControlsFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = LANDING;
}

void ControlsFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = DISARMING;
}

void ControlsFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = MANUAL;
}

void ControlsFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}
