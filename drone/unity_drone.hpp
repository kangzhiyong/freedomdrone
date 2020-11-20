#pragma once

#include "drone.hpp"

class UnityDrone :public Drone
{
    /*
    UnityDrone class adds additional low - level capabilities to control the
    Unity simulation version of the drone
    */
public:
    UnityDrone(MavlinkConnection* conn) :Drone(conn) {}
    point3D local_position_target();
    void set_local_position_target(point3D target);
    point3D local_velocity_target();
    void set_local_velocity_target(point3D target);
    point3D local_acceleration_target();
    void set_local_acceleration_target(point3D target);
    point3D attitude_target();
    void set_attitude_target(point3D target);
    point3D body_rate_target();
    void set_body_rate_target(point3D target);
    float threshold_horizontal_error();
    void set_threshold_horizontal_error(float threshold);
    float threshold_vertical_error();
    void set_threshold_vertical(float threshold);
    clock_t threshold_time();
    void set_threshold_time(clock_t threshold);
    void load_test_trajectory(vector<point3D>& position_trajectory, vector<float>& time_trajectory, vector<float>& yaw_trajectory);
    float calculate_horizontal_error();
    float calculate_vertical_error();
    void print_mission_score();
    void check_mission_success();
    void cmd_position(float target_north, float target_east, float target_down, float yaw) {}
private:
    float _target_north{ 0.0 };
    float _target_east{ 0.0 };
    float _target_down{ 0.0 };
    time_t _target_position_time;

    float _target_velocity_north{ 0.0 };
    float _target_velocity_east{ 0.0 };
    float _target_velocity_down{ 0.0 };
    time_t _target_velocity_time;

    float _target_acceleration_north{ 0.0 };
    float _target_acceleration_east{ 0.0 };
    float _target_acceleration_down{ 0.0 };
    time_t _target_acceleration_time;

    float _target_roll{ 0.0 };
    float _target_pitch{ 0.0 };
    float _target_yaw{ 0.0 };
    time_t _target_attitude_time;

    float _target_roll_rate{ 0.0 };
    float _target_pitch_rate{ 0.0 };
    float _target_yaw_rate{ 0.0 };
    time_t _target_body_rate_time;

    // Used for the autograder
    vector<float> all_horizontal_errors;
    float _threshold_horizontal_error{ 2.0 };
    vector<float> all_vertical_errors;
    float _threshold_vertical_error{ 1.0 };
    vector<clock_t> all_times;
    time_t _threshold_time;
    float _average_horizontal_error{ 0.0 };
    float _maximum_horizontal_error{ 0.0 };
    float _average_vertical_error{ 0.0 };
    float _maximum_vertical_error{ 0.0 };
    clock_t _mission_time;
    clock_t _time0;
    bool _mission_success{ true };
};
