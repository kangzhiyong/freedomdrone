// freedomdrone.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once

#include <iostream>
using namespace std;

#include "drone.hpp"
#include "mavlink_connection.hpp"
#include "free_point.hpp"
#include "free_data.hpp"
#include "outer_controller.hpp"
#include "inner_controller.hpp"
#include "msgpack.hpp"

/*
# NOTE: a waypoint here is defined as[North, East, Down]

###### EXAMPLES ######
#
# here are a set of example waypoint sets that you might find useful for testing.
# each has a bit of a description to help with the potential use case for the waypoint set.
#
# NOTE: the waypoint lists are defined as a list of lists, which each entry in a list of the
# [North, East, Down] to command.  Also recall for the crazyflie, North and East are defined
# by the starting position of the drone(straight and left, respectively), not world frame North
#and East.
#
###### ######## ######

######
# 1. have the crazyflie hover in a single place.
#
# For this to work best, make sure to comment out the waypoint transition code(see block comment
# in `local_position_callback`) to ensure that the crazyflie attempts to hold this position.
######

WAYPOINT_LIST = [[0.0, 0.0, -0.5]]


######
# 2. there and back.
#
# Simple 2 point waypoint path to go away and come back.
######

# WAYPOINT_LIST = [
#     [1.5, 0.0, -0.5],
#     [0.0, 0.0, -0.5]
#     ]


######
# 3. simple box.
#
# A simple box, much like what you flew for the backyard flyer examples.
######

                # WAYPOINT_LIST = [
#     [1.0, 0.0, -0.5],
#     [1.0, 1.0, -0.5],
#     [0.0, 1.0, -0.5],
#     [0.0, 0.0, -0.5]
#     ]
*/

// that height to which the drone should take off
#define TAKEOFF_ALTITUDE 0.5

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
    void find_closest_node(vector<point3D> nodes, point3D p, point3D &p_min);
    void send_waypoints(vector<point3D> points);
    void plan_path();
};

class AttitudeFlyer : public Drone
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
    InnerLoopController<float> _inner_controller;

    /*velocity commands
        # recall that the first level of controller computes velocity commands, and then
        # those commands are converted to attitude commands
        # this is done in 2 separate callbacks as the computation of the velocity command does not
        # need to happen at the same time as the attitude command computation
     */  
    point3D _velocity_cmd{ 0, 0, 0 };
public:
    AttitudeFlyer(MavlinkConnection* conn);
    void local_position_callback();
    void velocity_callback();
    void state_callback();
    void check_and_increment_waypoint();
    point3D run_outer_controller();
    point3D run_inner_controller();
    void calculate_box();
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start_drone();
};

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

class TrajectoryHandler
{
public:
    TrajectoryHandler(){}
    void _load_trajectory(string filename);
    void get_next_point(float inflight_time, point3D &position_cmd, point3D &velocity_cmd);
    bool is_trajectory_completed(float inflight_time);
private:
    vector<float> _rel_times; // list of the [N, E, D] positions for the trajectory in
    vector<point3D> _positions; // list of the relative times for the trajectory in decimal seconds
};

class TrajectoryVelocityFlyer : public Drone
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
    // get the trajectory handler
    TrajectoryHandler _traj_handler;
    float _start_time{0.0};  // this is the time that the flight started -> will be set on takeoff

public:
    TrajectoryVelocityFlyer(MavlinkConnection* conn);
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

