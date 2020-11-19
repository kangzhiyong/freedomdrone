#include "unity_drone.hpp"

point3D UnityDrone::local_position_target()
{
    return { _target_north, _target_east, _target_down };
}

void UnityDrone::set_local_position_target(point3D target)
{
    // Pass the local position target to the drone (not a command)
    _target_north = target[0];
    _target_east = target[1];
    _target_down = target[2];
    local_position_target(target[0], target[1], target[2], 0);

    // Check for current xtrack error
    if (_time0 == 0)
    {
        _time0 = clock();
    }

    _horizontal_error = calculate_horizontal_error();
    all_horizontal_errors.push_back(_horizontal_error);

    _vertical_error = calculate_vertical_error();
    all_vertical_errors.push_back(_vertical_error);
    _mission_time = clock() - _time0;
    all_times.push_back(_mission_time);
    check_mission_success();
}

point3D UnityDrone::local_velocity_target()
{
    return { _target_velocity_north, _target_velocity_east, _target_velocity_down };
}

void UnityDrone::set_local_velocity_target(point3D target)
{
    // Pass the local velocity target to the drone (not a command)
    _target_velocity_north = target[0];
    _target_velocity_east = target[1];
    _target_velocity_down = target[2];
    local_velocity_target(target[0], target[1], target[2], 0);
}

point3D UnityDrone::local_acceleration_target()
{
    return { _target_acceleration_north, _target_acceleration_east, _target_acceleration_down };
}

void UnityDrone::set_local_acceleration_target(point3D target)
{
    _target_acceleration_north = target[0];
    _target_acceleration_east = target[1];
    _target_acceleration_down = target[2];
    local_acceleration_target(target[0], target[1], target[2], 0);
}
point3D UnityDrone::attitude_target()
{
    return { _target_roll, _target_pitch, _target_yaw };
}

void UnityDrone::set_attitude_target(point3D target)
{
    // Pass the attitude target to the drone (not a command)
    _target_roll = target[0];
    _target_pitch = target[1];
    _target_yaw = target[2];
    attitude_target(target[0], target[1], target[2], 0);
}

point3D UnityDrone::body_rate_target()
{
    return { _target_roll_rate, _target_pitch_rate, _target_yaw_rate };
}

void UnityDrone::set_body_rate_target(point3D target)
{
    // Pass the local position target to the drone (not a command)
    _target_roll_rate = target[0];
    _target_pitch_rate = target[1];
    _target_yaw_rate = target[2];
    body_rate_target(target[0], target[1], target[2], 0);
}

float UnityDrone::threshold_horizontal_error()
{
    // Maximum allowed xtrack error on the mission
    return _threshold_xtrack;
}

void UnityDrone::set_threshold_horizontal_error(float threshold)
{
    if (threshold > 0.0)
    {
        _threshold_horizontal_error = threshold;
    }
    else
    {
        cout << "Horizontal error threshold must be greater than 0.0" << endl;
    }
}

float UnityDrone::threshold_vertical_error()
{
    // Maximum allowed xtrack error on the mission
    return _threshold_vertical_error;
}

void UnityDrone::set_threshold_vertical(float threshold)
{
    if (threshold > 0.0)
    {
        _threshold_vertical_error = threshold;
    }
    else
    {
        cout << "Vertical error threshold must be greater than 0.0" << endl;
    }
}

clock_t UnityDrone::threshold_time()
{
    // Maximum mission time
    return _threshold_time;
}

void UnityDrone::set_threshold_time(clock_t threshold)
{
    if (threshold > 0.0)
    {
        _threshold_time = threshold;
    }
    else
    {
        cout << "Time threshold must be greater than 0.0" << endl;
    }
}

void UnityDrone::load_test_trajectory(float time_mult, vector<point3D>& position_trajectory, vector<time_t>& time_trajectory, vector<point3D>& yaw_trajectory)
{
    /*Loads the test_trajectory.txt

    Args :
        time_mult : a multiplier to decrease the total time of the trajectory
    */
    vector<point<float, 4>> data = FreeData<float>::loadtxt('test_trajectory.txt', delimiter = ',', dtype = 'Float64');
    time_t current_time = time(0);
    for (size_t i = 0; i < data.size(); i++)
    {
        position_trajectory.push_back(data[i][1], data[i][2], data[i][3]);
        time_trajectory.push_back(data[i][0] * time_mult + current_time);
    }

    for (size_t i = 0; i < (position_trajectory.size() - 1); i++)
    {
        yaw_trajectory.push_back(atan2(position_trajectory[i + 1][1] - position_trajectory[i][1], position_trajectory[i + 1][0] - position_trajectory[i][0]));
    }
    yaw_trajectory.push_back(yaw_trajectory[yaw_trajectory.size() - 1]);
}

float UnityDrone::calculate_horizontal_error()
{
    //Calcuate the error beteween the local position and target local position
    point2D target_position({ _target_north, _target_east });
    return target_position.distance({ local_position()[0], local_position()[1] });
}

float UnityDrone::calculate_vertical_error()
{
    // Calculate the error in the vertical direction
    return abs(_target_down - local_position()[2]);
}

void UnityDrone::print_mission_score()
{
    // Prints the maximum xtrack error, total time, and mission success
    print('Maximum Horizontal Error: ', _maximum_horizontal_error);
    print('Maximum Vertical Error: ', _maximum_vertical_error);
    print('Mission Time: ', _mission_time);
    print('Mission Success: ', _mission_success);
}

void UnityDrone::check_mission_success()
{
    // Check the mission success criterion (xtrack and time)
    if (_horizontal_error > _maximum_horizontal_error)
    {
        _maximum_horizontal_error = _horizontal_error;
        if (_maximum_horizontal_error > _threshold_horizontal_error)
        {
            _mission_success = false;
        }
    }

    if (_vertical_error > _maximum_vertical_error)
    {
        _maximum_vertical_error = _vertical_error;
        if (_maximum_vertical_error > _threshold_vertical_error)
        {
            _mission_success = false;
        }
    }

    if (_mission_time > _threshold_time)
    {
        _mission_success = false;
    }
}