#pragma once

#include "common_flyer.hpp"

class VelocityFlyer : public Drone
{
    /*Implementation of the class to control a drone using attitude commands.

        uses both the inner and outer controller to compute the desired commands to send to the
        drone when each respective callback is triggered.
    */
protected:
    queue<point3D> all_waypoints;
    vector<States> check_state;
    States flight_state{ MANUAL };
    CallBackFunc m_pCallBackFunc{ nullptr };
    point3D target_position{ 0, 0, 0 };
    point3D target_velocity{ 0, 0, 0 };
    bool in_mission{ true };
    OuterLoopController<float> _outer_controller;
public:
    VelocityFlyer(MavlinkConnection* conn);
    void local_position_callback();
    void velocity_callback();
    void state_callback();
    void check_and_increment_waypoint();
    point3D run_outer_controller();
    void calculate_box();
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start_drone();
};
