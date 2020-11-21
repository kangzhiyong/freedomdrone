#pragma once

#include "msgpack.hpp"

#include "free_data.hpp"
#include "drone.hpp"

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
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start_drone();
    void find_closest_node(vector<point3D> nodes, point3D p, point3D &p_min);
    void send_waypoints(vector<point3D> points);
    void plan_path();
};

class MSGPoint
{
private:
    int x;
    int y;
    int z;
    int a;
public:
    MSGPoint(point3D p)
    {
        x = p[0];
        y = p[1];
        z = p[2];
        a = 0;
    }
    MSGPACK_DEFINE(x, y, z, a);
};