#include "backyard_flyer.hpp"
#include "free_utils.hpp"

BackyardFlyer::BackyardFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)()) & BackyardFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)()) & BackyardFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)()) & BackyardFlyer::state_callback));
}

void BackyardFlyer::local_position_callback()
{
    if (flight_state == TAKEOFF)
    {
        if (-1.0 * local_position()[2] > 0.95 * target_position[2])
        {
            calculate_box();
            waypoint_transition();
        }
    }
    else if (flight_state == WAYPOINT)
    {
        point3D lp = local_position();
        if (norm(target_position - lp) < 1.0)
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

void BackyardFlyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if ((global_position()[2] - global_home()[2] < 0.1)
            && (abs(local_position()[2]) < 0.01))
        {
            disarming_transition();
        }
    }
}

void BackyardFlyer::state_callback()
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

void BackyardFlyer::calculate_box()
{
    cout << "Setting Home" << endl;
    all_waypoints.push({ 10.0, 0.0, 3.0 });
    all_waypoints.push({ 10.0, 10.0, 3.0 });
    all_waypoints.push({ 0.0, 10.0, 3.0 });
    all_waypoints.push({ 0.0, 0.0, 3.0 });
}

void BackyardFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_position(global_position()[0], global_position()[1], global_position()[2]);
    flight_state = ARMING;
}

void BackyardFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = 3.0;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = TAKEOFF;
}

void BackyardFlyer::waypoint_transition()
{
    cout << "waypoint transition" << endl;
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    cmd_position(target_position[0], target_position[1], target_position[2], 0.0);
    flight_state = WAYPOINT;
}

void BackyardFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = LANDING;
}

void BackyardFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = DISARMING;
}

void BackyardFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = MANUAL;
}

void BackyardFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}
