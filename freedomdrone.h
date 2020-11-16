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

class MotionPlanning : public Drone
{
protected:
    queue<point3D> all_waypoints;
    vector<States> check_state;
    States flight_state{ MANUAL };
    CallBackFunc m_pCallBackFunc{ nullptr };
    point3D target_position{ 0, 0, 0 };
    bool in_mission{ true };
public:
    MotionPlanning(MavlinkConnection* conn);
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
    void send_waypoints();
    void plan_path();
};