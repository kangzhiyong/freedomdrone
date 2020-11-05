// freedomdrone.cpp: 定义应用程序的入口点。
//

#include "freedomdrone.h"

UpAndDownFlyer::UpAndDownFlyer(MavlinkConnection *conn): Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)())&UpAndDownFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)())&UpAndDownFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)())&UpAndDownFlyer::state_callback));
    
}

void UpAndDownFlyer::local_position_callback()
{
    if (flight_phase == TAKEOFF) {
        float altitude = -1.0 * local_position()[2];
        if (altitude > 0.95 * target_position[2]) {
            landing_transition();
        }
    }
}

void UpAndDownFlyer::velocity_callback()
{
    if (flight_phase == LANDING)
    {
        if (global_position()[2] - global_home()[2] < 0.1 && abs(local_position()[2]) < 0.01)
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
    if (flight_phase == MANUAL) {
        arming_transition();
    }
    else if (flight_phase == ARMING && armed())
    {
        takeoff_transition();
    }
    else if (flight_phase == DISARMING && !armed())
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
    flight_phase = TAKEOFF;
}

void UpAndDownFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    release_control();
    stop();
    flight_phase = MANUAL;
    in_mission = false;
}

void UpAndDownFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_position(global_position()[0], global_position()[1], global_position()[2]);
    flight_phase = ARMING;
}

void UpAndDownFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_phase = LANDING;
}

void UpAndDownFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    flight_phase = DISARMING;
}

void UpAndDownFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}

