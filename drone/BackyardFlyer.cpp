#include "BackyardFlyer.hpp"
#include "DroneUtils.hpp"

BackyardFlyer::BackyardFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(MessageIDs::LOCAL_POSITION, ((void (Drone::*)()) & BackyardFlyer::local_position_callback));
    register_callback(MessageIDs::LOCAL_VELOCITY, ((void (Drone::*)()) & BackyardFlyer::velocity_callback));
    register_callback(MessageIDs::STATE, ((void (Drone::*)()) & BackyardFlyer::state_callback));
}

void BackyardFlyer::local_position_callback()
{
    if (flight_state == States::TAKEOFF)
    {
        if (1.0 * local_position()[2] > 0.95 * target_position[2])
        {
            calculate_box();
            waypoint_transition();
        }
    }
    else if (flight_state == States::WAYPOINT)
    {
        if ((target_position - local_position()).mag() < 1.0)
        {
            if (all_waypoints.size() > 0)
            {
                waypoint_transition();
            }
            else if (local_velocity().mag() < 1.0)
            {
                landing_transition();
            }
        }
    }
}

void BackyardFlyer::velocity_callback()
{
    if (flight_state == States::LANDING)
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
        if (flight_state == States::MANUAL)
        {
            arming_transition();
        }
        else if (flight_state == States::ARMING && armed())
        {
            takeoff_transition();
        }
        else if (flight_state == States::DISARMING && !armed() && !guided())
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
    flight_state = States::ARMING;
}

void BackyardFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = 3.0;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = States::TAKEOFF;
}

void BackyardFlyer::waypoint_transition()
{
    cout << "waypoint transition" << endl;
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    cmd_position(target_position[0], target_position[1], target_position[2], 0.0);
    flight_state = States::WAYPOINT;
}

void BackyardFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = States::LANDING;
}

void BackyardFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = States::DISARMING;
}

void BackyardFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = States::MANUAL;
}

void BackyardFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}
