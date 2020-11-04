// freedomdrone.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once

#include <iostream>
using namespace std;

#include "drone.hpp"
#include "mavlink_connection.hpp"
#include "free_point.hpp"

typedef void (*CallBackFunc)(void *userData, double lo, double la, double alt);

enum States
{
    MANUAL = 0,
    ARMING,
    TAKEOFF,
    WAYPOINT,
    LANDING,
    DISARMING,
    PLANNING
};

class UpAndDownFlyer:public Drone
{
protected:
    CallBackFunc m_pCallBackFunc;
    point4D target_position{0, 0, 0, 0};
    bool in_mission{true};
    States flight_phase{MANUAL};
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
