#pragma once

#include <queue>
#include <vector>
using  namespace::std;
#include "msgpack.hpp"
#include "Point.hpp"
#include "UnityDrone.hpp"
#include "NonlinearController.hpp"
#include "BaseQuadEstimator.hpp"
#include "SearchAlgorithm.hpp"

#define TAKEOFF_ALTITUDE -10
#define RATE_100_HZ 100
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz

class ControlsFlyer:public UnityDrone
{
protected:
    CallBackFunc m_pCallBackFunc{nullptr};
    V3F target_position{0, 0, 0};
    bool in_mission{true};
    States flight_state{ States::MANUAL};
    NonlinearController controller;
    queue<V3F> all_waypoints;
    vector<States> check_state;
    V3F local_position_target{0, 0, 0};
    V3F local_velocity_target{0, 0, 0};
    V3F _attitude_target{ 0, 0, 0 };
    vector<V3F> position_trajectory;
    vector<float> yaw_trajectory;
    vector<float> time_trajectory;
    V3F local_acceleration_target;
    float yaw_target;
    float thrust_cmd;
    V3F body_rate_target;
    int waypoint_number;
    time_t _start_time;
    bool in_planning{ false };
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
    void gps_sensor_callback();
    void imu_sensor_callback();

    void find_closest_node(vector<V3F> nodes, V3F p, V3F& p_min);
    void send_waypoints(vector<V3F> points);
    void plan_path();
    void command_ack_callback();

    clock_t lastPrediction;
    clock_t nextPrediction;
    shared_ptr<BaseQuadEstimator> estimator;
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
