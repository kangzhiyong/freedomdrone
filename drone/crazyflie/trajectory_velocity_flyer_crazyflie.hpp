#pragma once
#include "common_flyer.hpp"

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
    time_t _start_time;  // this is the time that the flight started -> will be set on takeoff

public:
    TrajectoryVelocityFlyer(MavlinkConnection* conn);
    void local_position_callback();
    void velocity_callback();
    void state_callback();
    void check_and_increment_waypoint();
    point3D run_outer_controller();
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start_drone();
};

