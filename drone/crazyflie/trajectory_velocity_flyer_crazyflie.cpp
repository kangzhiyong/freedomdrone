#include "trajectory_velocity_flyer_crazyflie.hpp"
#include "free_data.hpp"

void TrajectoryHandler::_load_trajectory(string filename)
{
    /*helper function to load in a trajectory file.

    given the filename (or path + name), load in the trajectory file defined as follows:
     - each row contains a trajectory point
     - each row consists of the following 4 comma separated values:
        - relative time (seconds into the flight)
        - north position (in [m])
        - east posiiton (in [m])
        - down position (in [m])

    Args:
        filename: the filename containing the trajectory information
    */

    // load the data in
    vector<point<float, 4>> data = FreeData<float>::loadtxt(filename, ",");
    for (int i = 0; i < data.size(); i++) {
        // first element in each row is the relative time for the trajectroy part
        _rel_times.push_back(data[i][0]);
        // elements 2, 3, and 4 are the [N, E, D] position for the timestamp
        _positions.push_back({ data[i][1], data[i][2], data[i][3] });
    }
}
void TrajectoryHandler::get_next_point(float inflight_time, point3D& position_cmd, point3D& velocity_cmd)
{
    /*get the position and velocity command for the given in flight time.

    given a in flight time (in seconds) determine what the position and velocity
    commands should be.

    Args:
        inflight_time: the time in flight (in seconds), NOT ABSOLUTE TIME
    */

    // get the index of the trajectory with the closest time
    vector<float> rel_times = _rel_times;
    for (int i = 0; i < rel_times.size(); i++) {
        cout << rel_times[i] << " " << inflight_time << endl;
        rel_times[i] = abs(rel_times[i] - inflight_time);
        cout << rel_times[i] << endl;
    }
    int ind_min = min_element(rel_times.begin(), rel_times.end()) - rel_times.begin();

    // get the time of this point
    float time_ref = _rel_times[ind_min];
    point3D p0, p1;
    float t0, t1;
    // create the position and velocity commands,
    // which depends on if current time is before or after the trajectory point
    if (ind_min == 0)
    {
        p0 = p1 = _positions[ind_min];
        t0 = 0.0;
        t1 = 1.0;
    }
    else if (inflight_time < time_ref)  // before the current point
    {
        p0 = _positions[ind_min - 1];
        p1 = _positions[ind_min];

        t0 = _rel_times[ind_min - 1];
        t1 = _rel_times[ind_min];
    }
    else  // after the current point
    {
        // now need to check if we are at the end of the file
        if (ind_min >= _positions.size() - 1)
        {
            p0 = _positions[ind_min];
            p1 = _positions[ind_min];

            t0 = 0.0;
            t1 = 1.0;
        }
        else
        {
            p0 = _positions[ind_min];
            p1 = _positions[ind_min + 1];

            t0 = _rel_times[ind_min];
            t1 = _rel_times[ind_min + 1];
        }

    }
    cout << ind_min << endl;
    p1.print();
    p0.print();
    // compute the position command (interpolating between points)
    point3D p2 = p1 - p0;
    position_cmd = p2 * (inflight_time - t0) / (t1 - t0) + p0;

    // compute the velocity command based on the desired points and time
    velocity_cmd = p2 / (t1 - t0);
}

bool TrajectoryHandler::is_trajectory_completed(float inflight_time)
{
    //check if the trajectory has been completed
    return (inflight_time > _rel_times[_rel_times.size() - 1]);
}

TrajectoryVelocityFlyer::TrajectoryVelocityFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)()) & TrajectoryVelocityFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)()) & TrajectoryVelocityFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)()) & TrajectoryVelocityFlyer::state_callback));
    string path = "../../data/line_traj.txt";
#ifdef WIN32
    path = "../../../data/line_traj.txt";
#endif
    _traj_handler._load_trajectory(path);
}

void TrajectoryVelocityFlyer::local_position_callback()
{
    if (flight_state == TAKEOFF)
    {
        if (abs(1.0 * local_position()[2]) > abs(0.95 * target_position[2]))
        {
            _start_time = time(0);
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
        # self.check_and_increment_waypoint()
        ########################################################################################*/

        // get the current in flight time
        time_t rel_time = time(0) - _start_time;
        // check if trajectory is completed
        if (_traj_handler.is_trajectory_completed((float)rel_time))
        {
            landing_transition();
            return;
        }
        // set the target position and velocity from the trajectory
        _traj_handler.get_next_point(rel_time, target_position, target_velocity);
        // run the outer loop controller(position controller->to velocity command)
        point3D vel_cmd = run_outer_controller();

        cmd_velocity(vel_cmd[0], vel_cmd[1], vel_cmd[2], 0.0);
    }
}

void TrajectoryVelocityFlyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if (abs(local_velocity()[2]) < 0.05)
        {
            disarming_transition();
        }
    }
}

void TrajectoryVelocityFlyer::state_callback()
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

point3D TrajectoryVelocityFlyer::run_outer_controller()
{
    /*helper function to run the outer loop controller.

        calls the outer loop controller to run the lateral position and altitude controllers to
        get the velocity vector to command.

    Returns :
        the velocity vector to command as[vn, ve, vd] in[m / s]
        numpy array of floats
    */

    point3D lateral_vel_cmd = _outer_controller.lateral_position_control(target_position, local_position(), target_velocity);
    float hdot_cmd = _outer_controller.altitude_control(-target_position[2], -local_position()[2], -target_velocity[2]);

    return { lateral_vel_cmd[0], lateral_vel_cmd[1], -hdot_cmd };
}

void TrajectoryVelocityFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_as_current_position();
    flight_state = ARMING;
}

void TrajectoryVelocityFlyer::takeoff_transition()
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

void TrajectoryVelocityFlyer::waypoint_transition()
{
    flight_state = WAYPOINT;
}

void TrajectoryVelocityFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    /*# NOTE: the current configuration has the controller command everything from takeoff to landing
    # to let the crazyflie handle landing, uncomment the followand change the conditions accordingly
    # in the velocity callback*/
    land();
    flight_state = LANDING;
}

void TrajectoryVelocityFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = DISARMING;
}

void TrajectoryVelocityFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = MANUAL;
}

void TrajectoryVelocityFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}


