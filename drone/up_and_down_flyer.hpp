#pragma once

#include "common_flyer.hpp"

class UpAndDownFlyer:public Drone
{
protected:
    CallBackFunc m_pCallBackFunc{nullptr};
    point3D target_position{0, 0, 0};
    bool in_mission{true};
    States flight_state{MANUAL};
public:
    UpAndDownFlyer(MavlinkConnection *conn);
    void local_position_callback();
    void velocity_callback();
    void state_callback();
    void landing_transition();
    void disarming_transition();
    void arming_transition();
    void takeoff_transition();
    void manual_transition();
    void start_drone();
};