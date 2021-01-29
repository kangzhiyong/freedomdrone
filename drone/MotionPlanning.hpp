#pragma once

#include "msgpack.hpp"

#include "FreeData.hpp"
#include "Drone.hpp"

class MotionPlanning : public Drone
{
protected:
    queue<V3F> all_waypoints;
    vector<States> check_state;
    States flight_state{ States::MANUAL };
    CallBackFunc m_pCallBackFunc{ nullptr };
    V3F target_position{ 0, 0, 0 };
    bool in_mission{ true };
    bool in_planning{ false };
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
    void find_closest_node(vector<V3F> nodes, V3F p, V3F &p_min);
    void send_waypoints(vector<V3F> points);
    void plan_path();
    void command_ack_callback();
};

class MSGPoint
{
private:
    int x;
    int y;
    int z;
    int a;
public:
    MSGPoint(V3F p)
    {
        x = p[0];
        y = p[1];
        z = p[2];
        a = 0;
    }
    MSGPACK_DEFINE(x, y, z, a);
};
