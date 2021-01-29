#pragma once
/*Controller that generates velocity commands

Class for the specific impelementation of a controller that will be comanding velocity commands.
Contains the solution for P controllers on altitudeand lateral position to commands velocities
to a drone over the wireless link.*/

#include "Point.hpp"
#include "DroneUtils.hpp"

template<typename coordinate_type>
class OuterLoopController
{
    /*controller class for computing velocity commands to control lateral position and altitude.

    solution implementation to a controller that computes velocity commands from position and
    altitude commands.
    */
public:
    typedef Point<coordinate_type, 3> PointType;
    OuterLoopController()
    {
        // define all the gains that will be needed
        _kp_pos = 0.4;  // gain for lateral position error
        _kp_alt = 0.1; // gain for altitude error

        // some limits to use
        _v_max = 0.3;     // the maximum horizontal velocity in[m / s]
        _hdot_max = 0.9;   // the maximum vertical velocity in[m / s]
    }
    PointType lateral_position_control(PointType pos_cmd, PointType pos, PointType vel_cmd)
    {
        /*compute the North and East velocity command to control the lateral position.

            Use a PID controller(or your controller of choice) to compute the North and East velocity
            commands to be send to the crazyflie given the commanded position and current position.

        Args :
            pos_cmd : the commanded position[north, east, down] in[m]
            pos : the current position[north, east, down] in[m]
            vel_cmd : the commanded velocity[vn, ve, vd] in[m / s]

        Returns :
            the velocity command as a 2 element numpy array[Vn, Ve] 
            array
        */

        // compute a[Vn, Ve] command
        //PointType pos_error = pos_cmd - pos;
        //PointType lateral_vel_cmd = pos_error * _kp_pos + vel_cmd;
        //lateral_vel_cmd = clip(lateral_vel_cmd, -_v_max, _v_max);

        //return lateral_vel_cmd;
        return { 0, 0, 0 };
    }

    coordinate_type altitude_control(coordinate_type alt_cmd, coordinate_type alt, coordinate_type hdot_cmd = 0.0)
    {
        /*compute the vertical velocity command to control the altitude.

            Use a PID controller(or your controller of choice) to compute the vertical velocity
            command to be send to the crazyflie given the commanded altitude and current altitude.

        Args :
            alt_cmd : the commanded altitude in[m](positive up)
            alt : the current altitude in[m](positive up)
            hdot_cmd : the commanded vertical velocity in[m / s](positive up)

        Returns :
            the vertical velocity to command
            float
        */

        // compute a[Vup] command
        hdot_cmd += _kp_alt * (alt_cmd - alt);
        hdot_cmd = clip(hdot_cmd, -_hdot_max, _hdot_max);
        return hdot_cmd;
    }
private:
    // define all the gains that will be needed
    coordinate_type _kp_pos; // gain for lateral position error
    coordinate_type _kp_alt;  // gain for altitude error

    // some limits to use
    coordinate_type _v_max;     // the maximum horizontal velocity in[m / s]
    coordinate_type _hdot_max;    // the maximum vertical velocity in[m / s]
};
