#include <msgpack.hpp>
#include <string>
#include <iostream>
#include <sstream>

#include "freedomdrone.h"
#include "free_utils.hpp"
#include "search_algorithm.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

UpAndDownFlyer::UpAndDownFlyer(MavlinkConnection *conn): Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)())&UpAndDownFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)())&UpAndDownFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)())&UpAndDownFlyer::state_callback));
    
}

void UpAndDownFlyer::local_position_callback()
{
    if (flight_state == TAKEOFF) {
        float altitude = -1.0 * local_position()[2];
        if (altitude > 0.95 * target_position[2]) {
            landing_transition();
        }
    }
}

void UpAndDownFlyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if ((global_position()[2] - global_home()[2] < 0.1)
            && (abs(local_position()[2]) < 0.01))
        {
            disarming_transition();
        }
    }
}

void UpAndDownFlyer::state_callback()
{
    if (!in_mission) {
        return;
    }
    if (flight_state == MANUAL) {
        arming_transition();
    }
    else if (flight_state == ARMING && armed())
    {
        takeoff_transition();
    }
    else if (flight_state == DISARMING && !armed())
    {
        manual_transition();
    }
}

void UpAndDownFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = 3.0;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = TAKEOFF;
}

void UpAndDownFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    release_control();
    stop();
    flight_state = MANUAL;
    in_mission = false;
}

void UpAndDownFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_position(global_position()[0], global_position()[1], global_position()[2]);
    flight_state = ARMING;
}

void UpAndDownFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = LANDING;
}

void UpAndDownFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    flight_state = DISARMING;
}

void UpAndDownFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}

BackyardFlyer::BackyardFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)()) & BackyardFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)()) & BackyardFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)()) & BackyardFlyer::state_callback));

}

void BackyardFlyer::local_position_callback()
{
    if (flight_state == TAKEOFF)
    {
        if (-1.0 * local_position()[2] > 0.95 * target_position[2])
        {
            calculate_box();
            waypoint_transition();
        }
    }
    else if (flight_state == WAYPOINT)
    {
        point3D lp = local_position();
        if (norm(target_position - lp) < 1.0)
        {
            if (all_waypoints.size() > 0)
            {
                waypoint_transition();
            }
            else if (norm(local_velocity()) < 1.0)
            {
                landing_transition();
            }
        }
    }
}

void BackyardFlyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if ((global_position()[2] - global_home()[2] < 0.1)
            && (abs(local_position()[2]) < 0.01))
        {
            disarming_transition();
        }
    }
}

void BackyardFlyer::state_callback()
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

void BackyardFlyer::calculate_box()
{
    cout << "Setting Home" << endl;
    all_waypoints.push({ 10.0, 0.0, 3.0 });
    all_waypoints.push({ 10.0, 10.0, 3.0 });
    all_waypoints.push({ 0.0, 10.0, 3.0 });
    all_waypoints.push({ 0.0, 0.0, 3.0 });
}

void BackyardFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_position(global_position()[0], global_position()[1], global_position()[2]);
    flight_state = ARMING;
}

void BackyardFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = 3.0;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = TAKEOFF;
}

void BackyardFlyer::waypoint_transition()
{
    cout << "waypoint transition" << endl;
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    cmd_position(target_position[0], target_position[1], target_position[2], 0.0);
    flight_state = WAYPOINT;
}

void BackyardFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = LANDING;
}

void BackyardFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = DISARMING;
}

void BackyardFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = MANUAL;
}

void BackyardFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}

MotionPlanning::MotionPlanning(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)()) & MotionPlanning::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)()) & MotionPlanning::velocity_callback));
    register_callback(STATE, ((void (Drone::*)()) & MotionPlanning::state_callback));

}

void MotionPlanning::local_position_callback()
{
    if (flight_state == TAKEOFF)
    {
        if (-1.0 * local_position()[2] > 0.95 * target_position[2])
        {
            waypoint_transition();
        }
    }
    else if (flight_state == WAYPOINT)
    {
        point3D lp = local_position();
        if (norm(target_position - lp) < 1.0)
        {
            if (all_waypoints.size() > 0)
            {
                waypoint_transition();
            }
            else if (norm(local_velocity()) < 1.0)
            {
                landing_transition();
            }
        }
    }
}

void MotionPlanning::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if ((global_position()[2] - global_home()[2] < 0.1)
            && (abs(local_position()[2]) < 0.01))
        {
            disarming_transition();
        }
    }
}

void MotionPlanning::state_callback()
{
    if (in_mission)
    {
        if (flight_state == MANUAL)
        {
            arming_transition();
        }
        else if (flight_state == ARMING && armed())
        {
            plan_path();
        }
        else if (flight_state == PLANNING)
        {
            takeoff_transition();
        }
        else if (flight_state == DISARMING && !armed() && !guided())
        {
            manual_transition();
        }
    }
}

void MotionPlanning::arming_transition()
{
    flight_state = ARMING;
    cout << "arming transition\r\n" << endl;
    arm();
    take_control();
}

void MotionPlanning::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    takeoff(target_position[2]);
    flight_state = TAKEOFF;
}

void MotionPlanning::waypoint_transition()
{
    flight_state = WAYPOINT;
    cout << "waypoint transition" << endl;
    if (all_waypoints.size() <= 0)
    {
        return;
    }
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    cmd_position(target_position[0], target_position[1], target_position[2], 0.0);
}

void MotionPlanning::landing_transition()
{
    flight_state = LANDING;
    cout << "landing transition" << endl;
    land();
}

void MotionPlanning::disarming_transition()
{
    flight_state = DISARMING;
    cout << "disarm transition" << endl;
    disarm();
    release_control();
}

void MotionPlanning::manual_transition()
{
    flight_state = MANUAL;
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
}

void MotionPlanning::start_drone()
{
    cout << "starting connection" << endl;
    start();
}

void MotionPlanning::find_closest_node(vector<point3D> nodes, point3D p, point3D &p_min)
{
    int i, index = 0;
    float dist, min_dist = 0;
    for (i = 0; i < nodes.size(); i++) {
        dist = p.distance(nodes[i]);
        if (i == 0 || dist < min_dist) {
            min_dist = dist;
            index = i;
        }
    }
    if (i > 0) {
        p_min = nodes[index];
    }
}

void MotionPlanning::send_waypoints(vector<point3D> points)
{
    // serialize the object into the buffer.
    // any classes that implements write(const char*,size_t) can be a buffer.
    cout << "Sending waypoints to simulator ..." << endl;
    vector<MSGPoint> msgPoint;
    for (size_t i = 0; i < points.size(); i++)
    {
        msgPoint.push_back(points[i]);
    }
    msgpack::sbuffer buffer;
    msgpack::pack(buffer, msgPoint);
    getConnection()->getMaster()->write(buffer.data(), buffer.size());
}

void MotionPlanning::plan_path()
{
    cout << "Searching for a path ..." << endl;
    float target_altitude = 5;
    float safety_distance = 5;
    
    target_position[2] = target_altitude;
    string path = "../../data/colliders.csv";
#ifdef WIN32
    path = "../../../data/colliders.csv";
#endif
    FreeData<float> data(path, ",");
    float lat0 = data.getLat();
    float lon0 = data.getLon();
    set_home_position(lon0, lat0, 0);
    _update_local_position(global_to_local(global_position(), global_home()));
    
    data.extract_polygons(safety_distance);
    data.sample(1000);
    data.create_graph(10);
    point3D start = local_position();
    point3D goal = global_to_local({-122.396428, 37.795128, target_altitude}, global_home());
    cout << "start and goal" << endl;
    start.print();
    goal.print();
    vector<FreeEdge<float, 3>> edges;
    vector<point3D> nodes;
    FreeGraph<float, 3> graph = data.getGraph();
    graph.getAllNodesAndEdges(nodes, edges);

    point3D start_new, goal_new;
    find_closest_node(nodes, start, start_new);
    find_closest_node(nodes, goal, goal_new);
    cout << "startnew and goalnew" << endl;
    start_new.print();
    goal_new.print();
    
    GirdCellType start_grid(start_new);
    GirdCellType goal_grid(goal_new);
    SearchAlgorithm a_start_graph(start_grid, goal_grid);
    a_start_graph.a_start_graph(graph);
    
    vector<point3D> path_points_prune;
    vector<point3D> path_points = a_start_graph.get_path_points();
    if (path_points.size() > 0)
    {
        a_start_graph.prune_path_by_collinearity(path_points, path_points_prune);
        send_waypoints(path_points_prune);
        for (int i = 0; i < path_points_prune.size(); i++) {
            path_points_prune[i].print();
            all_waypoints.push({ path_points_prune[i][0], path_points_prune[i][1], path_points_prune[i][2] });
        }
        flight_state = PLANNING;
    }
}

AttitudeFlyer::AttitudeFlyer(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (Drone::*)()) & AttitudeFlyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (Drone::*)()) & AttitudeFlyer::velocity_callback));
    register_callback(STATE, ((void (Drone::*)()) & AttitudeFlyer::state_callback));

}

void AttitudeFlyer::local_position_callback()
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
    else if (flight_state == WAYPOINT || flight_state == TAKEOFF || flight_state == LANDING)
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

        // run the outer loop controller(position controller->to velocity command)
        _velocity_cmd = run_outer_controller();

        /*# NOTE: not sending the velocity command here!
        # this just sets the velocity command, which is used in the velocity callback, which
        # sends the attitude commands to the drone.*/
    }
}

void AttitudeFlyer::velocity_callback()
{
    if (flight_state == LANDING)
    {
        if (-1.0 * local_position()[2] < 0.1 && abs(local_velocity()[2]) < 0.05)
        {
            disarming_transition();
        }
    }

    /*# NOTE : this will run your controller during takeoff and landing.
      # to disable that functionality, you will need to remove those conditions from this if statement
    */
    if (flight_state == WAYPOINT || flight_state == TAKEOFF || flight_state == LANDING)
    {
        // run the inner loop controller
        point3D cmd = run_inner_controller();

        // NOTE: yaw control is not implemented, just commanding 0 yaw;
        cmd_attitude(cmd[0], cmd[1], 0.0, cmd[2]);
    }
}

void AttitudeFlyer::state_callback()
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

void AttitudeFlyer::check_and_increment_waypoint()
{
    /*helper function to handle waypoint checks and transitions

    check if the proximity condition has been met for a waypointand
    transition the waypoint as needed.
    if there are no more waypoints, trigger the landing transition.
     */

     /*# NOTE: depending on how aggressive of paths you are flying, and how reliably you want
     # them to be flown, you may want to add the vertical axis to the distance check.*/
    if (norm(target_position - local_position()) < 0.2)
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

point3D AttitudeFlyer::run_outer_controller()
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

point3D AttitudeFlyer::run_inner_controller()
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
    cout << "Setting Home" << endl;
    all_waypoints.push({ 0.0, 0.0, -0.5 });
}

void AttitudeFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_as_current_position();
    flight_state = ARMING;
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
    flight_state = TAKEOFF;
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
    flight_state = WAYPOINT;
}

void AttitudeFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    /*# NOTE: the current configuration has the controller command everything from takeoff to landing
    # to let the crazyflie handle landing, uncomment the followand change the conditions accordingly
    # in the velocity callback
    # land()*/
    flight_state = LANDING;
}

void AttitudeFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = DISARMING;
}

void AttitudeFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = MANUAL;
}

void AttitudeFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}