#include "velocity_flyer_crazyflie.hpp"

VelocityFlyer::VelocityFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)()) & VelocityFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)()) & VelocityFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)()) & VelocityFlyer::state_callback));

}

void VelocityFlyer::local_position_callback()
{
    if (flight_state == TAKEOFF)
    {
        if (-1.0 * local_position()[2] > -0.95 * target_position[2])
        {
            calculate_box();
            waypoint_transition();
        }
    }
    /*  # NOTE : as configured this controller handles the takeoff and landing conditions as well as waypoint flight.
        # however, you can configure it to let the crazyflie control it by uncommenting the
        # `self.land()` and `self.takeoff()` functions in their respective transition functions
        # and by removing those states from this elif line.
    */
    else if (flight_state == WAYPOINT /*|| flight_state == TAKEOFF || flight_state == LANDING*/)
    {
        /*  # DEBUG
            # print("curr pos: ({:.2f}, {:.2f}, {:.2f}), desired pos: ({:.2f}, {:.2f}, {:.2f})".format(
            #     self.local_position[0], self.local_position[1], self.local_position[2],
            #     self._target_position[0], self._target_position[1], self._target_position[2]))

        ########################### Waypoint Incrementing Block ################################
        #
        # NOTE: comment out this block of code if you wish to have the crazyflie simply hold
        # its first waypoint position.
        # This is a good way to be able to test your initial set of gains without having to
        # worry about your crazyflie flying away too quickly.
        #
        ########################################################################################*/
        check_and_increment_waypoint();
        // run the outer loop controller(position controller->to velocity command)
        point3D vel_cmd = run_outer_controller();
        cmd_velocity(vel_cmd[0], vel_cmd[1], vel_cmd[2], 0.0);
    }
}

void VelocityFlyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if (abs(local_velocity()[2]) < 0.05)
        {
            disarming_transition();
        }
    }
}

void VelocityFlyer::state_callback()
{
    if (in_mission)
    {
        if (flight_state == MANUAL)
        {
            arming_transition();
        }
        else if (flight_state == ARMING && armed())
        {
            takeoff_transition();
        }
        else if (flight_state == DISARMING && !armed() && !guided())
        {
            manual_transition();
        }
    }
}

void VelocityFlyer::check_and_increment_waypoint()
{
    /*helper function to handle waypoint checks and transitions

    check if the proximity condition has been met for a waypointand
    transition the waypoint as needed.
    if there are no more waypoints, trigger the landing transition.
     */

     /*# NOTE: depending on how aggressive of paths you are flying, and how reliably you want
     # them to be flown, you may want to add the vertical axis to the distance check.*/
    point3D lp = local_position();
    if (norm(target_position - lp) < 0.2)
    {
        if (all_waypoints.size() > 0)
        {
            waypoint_transition();
        }
        else if (norm(local_velocity()) < 0.2)
        {
            landing_transition();
        }
    }
}

point3D VelocityFlyer::run_outer_controller()
{
    /*helper function to run the outer loop controller.

        calls the outer loop controller to run the lateral position and altitude controllers to
        get the velocity vector to command.

    Returns :
        the velocity vector to command as[vn, ve, vd] in[m / s]
        numpy array of floats
    */

    point3D lateral_vel_cmd = _outer_controller.lateral_position_control(target_position, local_position(), target_velocity);
    float hdot_cmd = _outer_controller.altitude_control(-target_position[2], -local_position()[2]);

    return { lateral_vel_cmd[0], lateral_vel_cmd[1], -hdot_cmd };
}

void VelocityFlyer::calculate_box()
{
    cout << "Setting Home" << endl;
    all_waypoints.push({ 0, 5, 3 });
    all_waypoints.push({ 5, 5, 3 });
    all_waypoints.push({ 5, 0, 3 });
    all_waypoints.push({ 0, 0, 3 });
}

void VelocityFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_as_current_position();
    flight_state = ARMING;
}

void VelocityFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = TAKEOFF_ALTITUDE;
    target_position[2] = target_altitude;

    /*# NOTE: the current configuration has the controller command everything from takeoff to landing
    # to let the drone handle takeoff, uncomment the follow and change the conditions accordingly
    # in the velocity callback*/
    takeoff(target_altitude);
    flight_state = TAKEOFF;
}

void VelocityFlyer::waypoint_transition()
{
    cout << "waypoint transition" << endl;
    if (all_waypoints.size() <= 0)
    {
        return;
    }
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    cmd_position(target_position[0], target_position[1], target_position[2], 0.0);
    flight_state = WAYPOINT;
}

void VelocityFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    /*# NOTE: the current configuration has the controller command everything from takeoff to landing
    # to let the crazyflie handle landing, uncomment the followand change the conditions accordingly
    # in the velocity callback*/
    land();
    flight_state = LANDING;
}

void VelocityFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = DISARMING;
}

void VelocityFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = MANUAL;
}

void VelocityFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}