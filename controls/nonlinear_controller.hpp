#pragma once

#include "free_point.hpp"
#include "Quaternion.hpp"

#define DRONE_MASS_KG 0.5
#define GRAVITY  -9.81
#define MAX_THRUST 10.0
#define MAX_TORQUE 1

extern point3D MOI;

class NonlinearController
{
private:
    float Kp_pos{ 6 }; //12.0,
    float Kp_vel{ 4 }; //8.0,
    float Kp_alt{ 4.0 };
    float Kp_hdot{ 1.5 }; //2.0,

    float Kp_roll{ 8 }; //8, 6.5,
    float Kp_pitch{ 8 }; //8, #6.5,
    float Kp_yaw{ 4.5 }; //4.5, #4.5,

    float Kp_p{ 20 }; //10,
    float Kp_q{ 20 }; //10,
    float Kp_r{ 5 }; //10,

    float max_tilt{ 1.0 };
    float max_ascent_rate{ 5 };
    float max_descent_rate{ 2 };
    float max_speed{ 5.0 };
    float max_accel{ 5.0 };
    // integral control
    float integrated_altitude_error{ 0 };
public:
    // Estimator state
    Quaternion<float> estAtt;
    V3F estVel;
    V3F estPos;
    V3F estOmega;

    NonlinearController() {}
    point3D trajectory_control(vector<point3D> position_trajectory, vector<float> yaw_trajectory, vector<float> time_trajectory,
                               float current_time, point3D& position_cmd, point3D& velocity_cmd, float& yaw_cmd);
    point2D lateral_position_control(point2D local_position_cmd, point2D local_velocity_cmd, point2D local_position, point2D local_velocity, point2D acceleration_ff);
    float altitude_control(float altitude_cmd, float vertical_velocity_cmd, float altitude, float vertical_velocity, point3D attitude, float acceleration_ff = 0.0);
    point2D roll_pitch_controller(point2D acceleration_cmd, point3D attitude, float thrust_cmd);
    point3D body_rate_control(point3D body_rate_cmd, point3D body_rate);
    float yaw_control(float yaw_cmd, float yaw);
    void UpdateEstimates(V3F pos, V3F vel, Quaternion<float> attitude, V3F omega)
};

