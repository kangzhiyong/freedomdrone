#pragma once

#include "common_flyer.hpp"

class BackyardFlyer : public Drone
{
protected:
    queue<point3D> all_waypoints;
    vector<States> check_state;
    States flight_state{ MANUAL };
    CallBackFunc m_pCallBackFunc{ nullptr };
    point3D target_position{ 0, 0, 0 };
    bool in_mission{ true };
public:
    BackyardFlyer(MavlinkConnection* conn);
    void local_position_callback();
    void velocity_callback();
    void state_callback();
    void calculate_box();
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start_drone();
};
