#pragma once
/*Controller class for computing attitude and thrust commands.
implementation of the inner loop controller which controls velocity through attitude and thrust commands.*/

#include "free_utils.hpp"
#include "free_point.hpp"

#define	DRONE_M	0.005			// [kg]
#define	GRAVITY_MAG 9.81      // [m / s ^ 2]->magnitude only
#define	MAX_THRUST_N 0.63     // the maximum amount of thrust the crazyflie can generate in[N] - DO NOT EDIT

template<typename coordinate_type>
class InnerLoopController
{
public:
    typedef point<coordinate_type, 3> PointType;
	InnerLoopController()
	{
        _kp_vel = 0.14;
        _kp_hdot = 1.0;
        _bank_max = 20 * M_DEG_TO_RAD;
        _haccel_max = 1.2;
	}

    PointType velocity_control(PointType vel_cmd, PointType vel)
    {
        /*compute attitude and thrust commands to achieve a commanded velocity vector.

            Use a PID controller(or your controller of choice) to compute the attitude(roll / pitch) to
            achieve the commanded(North, East) velocity and compute a normalized thrust to achieve the
            commanded down velocity.

        Args :
            vel_cmd : the commanded velocity vector as a numpy array[Vnorth, Veast, Vdown] in[m / s]
            vel : the current velocity vector as a numpy array[Vnorth, Veast, Vdown] in[m / s]
        */

        // change down velocities to up hdots
        coordinate_type hdot_cmd = -vel_cmd[2];
        coordinate_type hdot = -vel[2];

        /* compute an attitude command from the given velocity command
           compute a normalized thrust from the given hdot command*/

        coordinate_type pitch = -_kp_vel * (vel_cmd[0] - vel[0]);  // note the sign change!Remember + pitch is up, meaning it will send out drone backwards!
        coordinate_type roll = _kp_vel * (vel_cmd[1] - vel[1]);

        // add some limits
        coordinate_type pitch_cmd = clip(pitch, -_bank_max, _bank_max);
        coordinate_type roll_cmd = clip(roll, -_bank_max, _bank_max);

        // compute acceleration and then thrust for vertical
        coordinate_type accel_cmd = _kp_hdot * (hdot_cmd - hdot);
        accel_cmd = clip(accel_cmd, -_haccel_max, _haccel_max);
        coordinate_type thrust_cmd_N = DRONE_M * (accel_cmd + GRAVITY_MAG) / (cos(pitch_cmd) * cos(roll_cmd)); // positive up

        // need to normalize the thrust
        coordinate_type thrust_cmd = thrust_cmd_N / MAX_THRUST_N;

        return { roll_cmd, pitch_cmd, thrust_cmd };
    }
private:
    // the gains that are needed
    coordinate_type _kp_vel;  // the gain on the velocity to get attitude
    coordinate_type _kp_hdot;  // the gain on the vertical velocity to get accel

    // some limits to use
    coordinate_type _bank_max;   // max bank(roll and pitch) angle - in radians
    coordinate_type _haccel_max;            // the maximum vertical acceleration in[m / s ^ 2]
};
