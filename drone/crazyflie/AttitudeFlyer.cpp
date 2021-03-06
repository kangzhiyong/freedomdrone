#include "AttitudeFlyer.hpp"

AttitudeFlyer::AttitudeFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(MessageIDs::LOCAL_POSITION, ((void (Drone::*)()) & AttitudeFlyer::local_position_callback));
    register_callback(MessageIDs::LOCAL_VELOCITY, ((void (Drone::*)()) & AttitudeFlyer::velocity_callback));
    register_callback(MessageIDs::STATE, ((void (Drone::*)()) & AttitudeFlyer::state_callback));
    register_callback(MessageIDs::COMMANDACKRESULT, ((void (Drone::*)()) & AttitudeFlyer::command_ack_callback));
}

void AttitudeFlyer::command_ack_callback()
{
    //if (!m_bControlStatus /*&& m_bTakeoffed*/)
    //{
    //    std::this_thread::sleep_for(std::chrono::seconds(5));
    //    cout << "cmd offboard on" << endl;
    //    getConnection()->make_command_flight_mode(FlightMode::Offboard);
    //    std::this_thread::sleep_for(std::chrono::seconds(1));
    //    m_bControlStatus = true;
    //}
}

void AttitudeFlyer::local_position_callback()
{
    if (flight_state == States::TAKEOFF)
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
    if (flight_state == States::WAYPOINT || flight_state == States::TAKEOFF || flight_state == States::LANDING)
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
        # self.check_and_increment_waypoint()
        ########################################################################################*/
        check_and_increment_waypoint();
        // run the outer loop controller(position controller->to velocity command)
        _velocity_cmd = run_outer_controller();

        /*# NOTE: not sending the velocity command here!
        # this just sets the velocity command, which is used in the velocity callback, which
        # sends the attitude commands to the drone.*/
    }
}

void AttitudeFlyer::velocity_callback()
{
    if (flight_state == States::LANDING)
    {
        if (-1.0 * local_position()[2] < 0.1 && abs(local_velocity()[2]) < 0.05)
        {
            disarming_transition();
        }
    }
    /*# NOTE : this will run your controller during takeoff and landing.
      # to disable that functionality, you will need to remove those conditions from this if statement
    */
    if (flight_state == States::WAYPOINT || flight_state == States::TAKEOFF || flight_state == States::LANDING)
    {
        // run the inner loop controller
        V3F cmd = run_inner_controller();
        // NOTE: yaw control is not implemented, just commanding 0 yaw;
        cmd_attitude(cmd[0], cmd[1], 0.0, cmd[2]);
    }
}

void AttitudeFlyer::state_callback()
{
    if (in_mission)
    {
        if (flight_state == States::MANUAL)
        {
            arming_transition();
        }
        else if (flight_state == States::ARMING && armed())
        {
            takeoff_transition();
        }
        else if (flight_state == States::DISARMING && !armed() && !guided())
        {
            manual_transition();
        }
    }
}

void AttitudeFlyer::check_and_increment_waypoint()
{
    /*helper function to handle waypoint checks and transitions

    check if the proximity condition has been met for a waypointand
    transition the waypoint as needed.
    if there are no more waypoints, trigger the landing transition.
     */

     /*# NOTE: depending on how aggressive of paths you are flying, and how reliably you want
     # them to be flown, you may want to add the vertical axis to the distance check.*/

    if ((target_position - local_position()).mag() < 0.2)
    {
        if (all_waypoints.size() > 0)
        {
            waypoint_transition();
        }
        else if (local_velocity().mag() < 0.2)
        {
            landing_transition();
        }
    }
}

V3F AttitudeFlyer::run_outer_controller()
{
    /*helper function to run the outer loop controller.

        calls the outer loop controller to run the lateral position and altitude controllers to
        get the velocity vector to command.

    Returns :
        the velocity vector to command as[vn, ve, vd] in[m / s]
        numpy array of floats
    */

    V3F lateral_vel_cmd = _outer_controller.lateral_position_control(target_position, local_position(), target_velocity);
    float hdot_cmd = _outer_controller.altitude_control(-target_position[2], -local_position()[2]);
    return { lateral_vel_cmd[0], lateral_vel_cmd[1], -hdot_cmd };
}

V3F AttitudeFlyer::run_inner_controller()
{
    /*helper function to run the inner loop controller.

    calls the inner loop controller to run the velocity controller to get the attitude and
    thrust commands.
    note that thrust in this case is a normalized value(between 0 and 1)

    Returns:
        a tuple of roll, pitch, and thrust commands
        a tuple(roll, pitch, thrust)
     */
    return _inner_controller.velocity_control(_velocity_cmd, local_velocity());
}

void AttitudeFlyer::calculate_box()
{
    cout << "calculate_box" << endl;
    all_waypoints.push({ 0, 5, TAKEOFF_ALTITUDE });
    all_waypoints.push({ 5, 5, TAKEOFF_ALTITUDE });
    all_waypoints.push({ 5, 0, TAKEOFF_ALTITUDE });
    all_waypoints.push({ 0, 0, TAKEOFF_ALTITUDE });
    all_waypoints.push({ 0, 0, 0 });
}

void AttitudeFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    arm();
    //set_home_as_current_position();
    flight_state = States::ARMING;
}

void AttitudeFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = TAKEOFF_ALTITUDE;
    target_position[2] = target_altitude;

    /*# NOTE: the current configuration has the controller command everything from takeoff to landing
    # to let the drone handle takeoff, uncomment the follow and change the conditions accordingly
    # in the velocity callback
    # takeoff(target_altitude)*/
    flight_state = States::TAKEOFF;
}

void AttitudeFlyer::waypoint_transition()
{
    cout << "waypoint transition" << endl;
    if (all_waypoints.size() <= 0)
    {
        return;
    }
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    flight_state = States::WAYPOINT;
}

void AttitudeFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    /*# NOTE: the current configuration has the controller command everything from takeoff to landing
    # to let the crazyflie handle landing, uncomment the followand change the conditions accordingly
    # in the velocity callback
    # land()*/
    flight_state = States::LANDING;
    //cout << target_position.str() << local_position().str() << local_velocity().str() << " " << local_velocity().mag() << endl;
}

void AttitudeFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    flight_state = States::DISARMING;
}

void AttitudeFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = States::MANUAL;
}

void AttitudeFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}
