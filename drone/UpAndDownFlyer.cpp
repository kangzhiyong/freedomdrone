#include "UpAndDownFlyer.hpp"

UpAndDownFlyer::UpAndDownFlyer(MavlinkConnection *conn): Drone(conn)
{
    // register all your callbacks here
    register_callback(MessageIDs::LOCAL_POSITION, ((void (Drone::*)())&UpAndDownFlyer::local_position_callback));
    register_callback(MessageIDs::LOCAL_VELOCITY, ((void (Drone::*)())&UpAndDownFlyer::velocity_callback));
    register_callback(MessageIDs::STATE, ((void (Drone::*)())&UpAndDownFlyer::state_callback));
    
}

void UpAndDownFlyer::local_position_callback()
{
    if (flight_state == States::TAKEOFF) {
        float altitude = -1.0 * local_position()[2];
        if (altitude > 0.95 * target_position[2]) {
            landing_transition();
        }
    }
}

void UpAndDownFlyer::velocity_callback()
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

void UpAndDownFlyer::state_callback()
{
    if (!in_mission) {
        return;
    }
    if (flight_state == States::MANUAL) {
        arming_transition();
    }
    else if (flight_state == States::ARMING && armed())
    {
        takeoff_transition();
    }
    else if (flight_state == States::DISARMING && !armed())
    {
        manual_transition();
    }
}

void UpAndDownFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = 3.0;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = States::TAKEOFF;
}

void UpAndDownFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    release_control();
    stop();
    flight_state = States::MANUAL;
    in_mission = false;
}

void UpAndDownFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_position(global_position()[0], global_position()[1], global_position()[2]);
    flight_state = States::ARMING;
}

void UpAndDownFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = States::LANDING;
}

void UpAndDownFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    flight_state = States::DISARMING;
}

void UpAndDownFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}
