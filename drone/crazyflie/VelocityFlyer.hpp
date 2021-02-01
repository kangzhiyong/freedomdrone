#pragma once

#include <queue>
#include <vector>
using  namespace::std;
#include "Point.hpp"
#include "Drone.hpp"
#include "OuterLoopController.hpp"

#define TAKEOFF_ALTITUDE -10

class VelocityFlyer : public Drone
{
    /*Implementation of the class to control a drone using attitude commands.

        uses both the inner and outer controller to compute the desired commands to send to the
        drone when each respective callback is triggered.
    */
protected:
    queue<V3F> all_waypoints;
    vector<States> check_state;
    States flight_state{ States::MANUAL };
    CallBackFunc m_pCallBackFunc{ nullptr };
    V3F target_position{ 0, 0, 0 };
    V3F target_velocity{ 0, 0, 0 };
    bool in_mission{ true };
    OuterLoopController<float> _outer_controller;
public:
    VelocityFlyer(MavlinkConnection* conn);
    void local_position_callback();
    void velocity_callback();
    void state_callback();
    void check_and_increment_waypoint();
    V3F run_outer_controller();
    void calculate_box();
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start_drone();
    void command_ack_callback();
};
