#pragma once

#include "Point.hpp"
#include "Quaternion.hpp"

extern V3F MOI;

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

    float Ki_alt{ 0 };

    float max_tilt{ 1.0 };
    float max_ascent_rate{ 5 };
    float max_descent_rate{ 2 };
    float max_speed{ 5.0 };
    float max_accel{ 5.0 };
    // integral control
    float integrated_altitude_error{ 0 };
public:
    NonlinearController() {}
    V3F trajectory_control(vector<V3F> position_trajectory, vector<float> yaw_trajectory, vector<float> time_trajectory,
                               float current_time, V3F& position_cmd, V3F& velocity_cmd, float& yaw_cmd);
    V3F lateral_position_control(V3F local_position_cmd, V3F local_velocity_cmd, V3F local_position, V3F local_velocity, V3F acceleration_ff);
    float altitude_control(float altitude_cmd, float vertical_velocity_cmd, float altitude, float vertical_velocity, SLR::Quaternion<float> attitude, float acceleration_ff = 0.0, float dt=0);
    V3F roll_pitch_controller(V3F acceleration_cmd, SLR::Quaternion<float> attitude, float thrust_cmd);
    V3F body_rate_control(V3F body_rate_cmd, V3F body_rate);
    float yaw_control(float yaw_cmd, float yaw);
    void UpdateEstimates(V3F pos, V3F vel, SLR::Quaternion<float> attitude, V3F omega);

    SLR::Quaternion<float> estAtt;
    V3F estVel;
    V3F estPos;
    V3F estOmega;
    
};

