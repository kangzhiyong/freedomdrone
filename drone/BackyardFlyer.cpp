#include "BackyardFlyer.hpp"
#include "DroneUtils.hpp"

BackyardFlyer::BackyardFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(MessageIDs::LOCAL_POSITION, ((void (Drone::*)()) & BackyardFlyer::local_position_callback));
    register_callback(MessageIDs::LOCAL_VELOCITY, ((void (Drone::*)()) & BackyardFlyer::velocity_callback));
    register_callback(MessageIDs::STATE, ((void (Drone::*)()) & BackyardFlyer::state_callback));
    register_callback(MessageIDs::COMMANDACKRESULT, ((void (Drone::*)()) & BackyardFlyer::command_ack_callback));
}

void BackyardFlyer::local_position_callback()
{
    if (flight_state == States::TAKEOFF)
    {
        if (abs(abs(local_position()[2]) - abs(target_position[2])) < 0.1)
        {
            calculate_box();
            waypoint_transition();
        }
    }
    else if (flight_state == States::WAYPOINT)
    {
        V3F t({ target_position[0], target_position[1], target_position[2] });
        if ((t - local_position()).mag3() < 1.0)
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
        else if (flight_state == States::LANDING)
        {
            if (!armed() && !guided())
            {
                stop();
                in_mission = false;
            }
        }
        else if (flight_state == States::DISARMING)
        {

        }
    }
}

void BackyardFlyer::calculate_box()
{
    cout << "calculate_box" << endl;
    //V4F cp = local_position();
    //all_waypoints.push(cp + V3F({ 10.0, 0.0, 0 }));
    //all_waypoints.push(cp + V3F({ 10.0, 10.0, 0 }));
    //all_waypoints.push(cp + V3F({ 0.0, 10.0, 0 }));
    //all_waypoints.push(cp + V3F({ 0.0, 0.0, 0 }));
    const float radius = 10.0f;
    const float step = 0.01f;
    for (float angle = 0.0f; angle <= 2.0f * M_PI; angle += step) {
        float x = radius * cosf(angle);
        float y = radius * sinf(angle);

        V4F pos({ x, y, -10.0f, 90.0f });
        all_waypoints.push(pos);
    }
}

void BackyardFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    //take_control();
    arm();
    flight_state = States::ARMING;
}

void BackyardFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    local_position().print();
    float target_altitude = local_position()[2] + 10.0;
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
    cmd_position(target_position);
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
    //release_control();
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

void BackyardFlyer::command_ack_callback()
{
    if (!m_bControlStatus && m_bTakeoffed)
    {
        cout << "cmd offboard on" << endl;
        cmd_position(local_position()[0], local_position()[1], -abs(target_position[2]), 0);
        getConnection()->make_command_flight_mode(FlightMode::Offboard);
        m_bControlStatus = true;
    }
}