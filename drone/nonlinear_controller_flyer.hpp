#pragma once

#include <queue>
#include <vector>
using  namespace::std;
#include "free_point.hpp"
#include "unity_drone.hpp"
#include "nonlinear_controller.hpp"

#define TAKEOFF_ALTITUDE 0.5

class ControlsFlyer:public UnityDrone
{
protected:
    CallBackFunc m_pCallBackFunc{nullptr};
    point3D target_position{0, 0, 0};
    bool in_mission{true};
    States flight_state{MANUAL};
    NonlinearController controller;
    queue<point3D> all_waypoints;
    vector<States> check_state;
    point3D local_position_target;
    point3D local_velocity_target;
    vector<point3D> position_trajectory;
    vector<float> yaw_trajectory;
    vector<float> time_trajectory;
    point2D local_acceleration_target;
    float yaw_target;
    float thrust_cmd;
    point3D body_rate_target;
    int waypoint_number;
    time_t _start_time;
public:
    ControlsFlyer(MavlinkConnection *conn);
    void position_controller();
    void attitude_controller();
    void bodyrate_controller();
    void attitude_callback();
    void gyro_callback();
    void local_position_callback();
    void check_and_increment_waypoint();
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
