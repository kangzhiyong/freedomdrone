#include "ControlsFlyer.hpp"
#include "DroneUtils.hpp"
#include "FreeData.hpp"
#include "QuadEstimatorEKF.hpp"

ControlsFlyer::ControlsFlyer(MavlinkConnection* conn) : UnityDrone(conn)
{
    // register all your callbacks here
    register_callback(MessageIDs::LOCAL_POSITION, ((void (Drone::*)()) & ControlsFlyer::local_position_callback));
    register_callback(MessageIDs::LOCAL_VELOCITY, ((void (Drone::*)()) & ControlsFlyer::velocity_callback));
    register_callback(MessageIDs::STATE, ((void (Drone::*)()) & ControlsFlyer::state_callback));
    register_callback(MessageIDs::ATTITUDE, ((void (Drone::*)()) & ControlsFlyer::attitude_callback));
    register_callback(MessageIDs::RAW_GYROSCOPE, ((void (Drone::*)()) & ControlsFlyer::gyro_callback));
    register_callback(MessageIDs::GPS_INPUT_SENSOR, ((void (Drone::*)()) & ControlsFlyer::gps_sensor_callback));
    register_callback(MessageIDs::RAW_IMU_SENSOR, ((void (Drone::*)()) & ControlsFlyer::imu_sensor_callback));
    register_callback(MessageIDs::COMMANDACKRESULT, ((void (Drone::*)()) & ControlsFlyer::command_ack_callback));

    lastPrediction = nextPrediction = clock();
    estimator.reset(new QuadEstimatorEKF());
}

void ControlsFlyer::position_controller()
{
//    float curr_time = time(0) - _start_time;
//    controller.trajectory_control(position_trajectory, yaw_trajectory, time_trajectory, curr_time, local_position_target, local_velocity_target, yaw_target);
    V3F acceleration_cmd = controller.lateral_position_control(local_position_target, local_velocity_target,
                                                               controller.estPos, controller.estVel, { 0, 0, 0 });
    local_acceleration_target = V3F({acceleration_cmd[0], acceleration_cmd[1], 0.0});
}

void ControlsFlyer::attitude_controller()
{
    thrust_cmd = controller.altitude_control(local_position_target[2], local_velocity_target[2],
                                             controller.estPos.z(), controller.estVel.z(), controller.estAtt, GRAVITY);
    V3F roll_pitch_rate_cmd = controller.roll_pitch_controller(local_acceleration_target, controller.estAtt, thrust_cmd);
    float yawrate_cmd = controller.yaw_control(yaw_target, controller.estAtt.Yaw());
    body_rate_target = V3F({roll_pitch_rate_cmd[0], roll_pitch_rate_cmd[1], yawrate_cmd});
}

void ControlsFlyer::bodyrate_controller()
{
    V3F moment_cmd = controller.body_rate_control(body_rate_target, gyro_raw());
    cmd_moment(moment_cmd[0], moment_cmd[1], moment_cmd[2], thrust_cmd);
}

void ControlsFlyer::attitude_callback()
{
    if (flight_state == States::WAYPOINT)
    {
        attitude_controller();
    }
}

void ControlsFlyer::gyro_callback()
{
    if (flight_state == States::WAYPOINT)
    {
        bodyrate_controller();
    }
}

void ControlsFlyer::local_position_callback()
{
    if (flight_state == States::TAKEOFF)
    {
        if (abs(abs(local_position()[2]) - abs(target_position[2])) < 0.1)
        {
            //_start_time = time(0);
             //calculate_box();
//            load_test_trajectory(position_trajectory, time_trajectory, yaw_trajectory);
//            for (int i = 0; i < position_trajectory.size(); i++) {
//                all_waypoints.push(position_trajectory[i]);
//            }
            //plan_path();
            waypoint_number = -1;
            waypoint_transition();
        }
    }
    else if (flight_state == States::WAYPOINT)
    {
//        if (time_trajectory.size() > 0 && waypoint_number >= 0 && ((time(0) - _start_time) > time_trajectory[waypoint_number]))
//        {
//            if (all_waypoints.size() > 0)
//            {
//                waypoint_transition();
//            }
//            else if (local_velocity().mag() < 1.0)
//            {
//                landing_transition();
//            }
//        }
        //check_and_increment_waypoint();
    }
}

void ControlsFlyer::check_and_increment_waypoint()
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

void ControlsFlyer::velocity_callback()
{
    if (flight_state == States::LANDING)
    {
        if (global_position()[2] - global_home()[2] < 0.1)
        {
            if (abs(local_position()[2]) < 0.01)
            {
                disarming_transition();
            }
        }
    }
    if (flight_state == States::WAYPOINT)
    {
        position_controller();
    }
}

void ControlsFlyer::state_callback()
{
    if (in_mission)
    {
        if (flight_state == States::MANUAL)
        {
            calculate_box();
            flight_state = States::PLANNING;
            /*if (!in_planning)
            {
                new thread(&ControlsFlyer::plan_path, this);
                in_planning = true;
            }*/
        }
        else if (flight_state == States::PLANNING)
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

void ControlsFlyer::calculate_box()
{
    cout << "calculate_box" << endl;
    //V3F cp = local_position();
    V3F cp({ 0, 0, TAKEOFF_ALTITUDE });
    all_waypoints.push(cp + V3F({ 10.0, 0.0, 0 }));
    all_waypoints.push(cp + V3F({ 10.0, 10.0, 0 }));
    all_waypoints.push(cp + V3F({ 0.0, 10.0, 0 }));
    all_waypoints.push(cp + V3F({ 0.0, 0.0, 0 }));
}

void ControlsFlyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    arm();
    //set_home_as_current_position();
    flight_state = States::ARMING;
}

void ControlsFlyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = TAKEOFF_ALTITUDE;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = States::TAKEOFF;
}

void ControlsFlyer::waypoint_transition()
{
    //cout << "waypoint transition" << endl;
    if (all_waypoints.size() <= 0)
    {
        return;
    }
    waypoint_number = waypoint_number + 1;
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print("target_position");
    set_local_position_target(target_position);
    flight_state = States::WAYPOINT;
}

void ControlsFlyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = States::LANDING;
}

void ControlsFlyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = States::DISARMING;
}

void ControlsFlyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = States::MANUAL;
}

void ControlsFlyer::start_drone()
{
    cout << "starting connection" << endl;
    start();
}

void ControlsFlyer::find_closest_node(vector<V3F> nodes, V3F p, V3F& p_min)
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

void ControlsFlyer::send_waypoints(vector<V3F> points)
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

void ControlsFlyer::plan_path()
{
    cout << "Searching for a path ..." << endl;
    float target_altitude = 5;
    float safety_distance = 5;

    target_position[2] = target_altitude;
    string path = "../../../../data/colliders.csv";
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
    V3F start = local_position();
    V3F goal({ 10, 10, 10 });
    //V3F goal = global_to_local({ -122.396428, 37.795128, target_altitude }, global_home());
    cout << "start and goal" << endl;
    start.print();
    goal.print();
    vector<FreeEdge<float, 3>> edges;
    vector<V3F> nodes;
    FreeGraph<float, 3> graph = data.getGraph();
    graph.getAllNodesAndEdges(nodes, edges);

    V3F start_new, goal_new;
    find_closest_node(nodes, start, start_new);
    find_closest_node(nodes, goal, goal_new);
    cout << "startnew and goalnew" << endl;
    start_new.print();
    goal_new.print();

    GirdCellType start_grid(start_new);
    GirdCellType goal_grid(goal_new);
    SearchAlgorithm a_start_graph(start_grid, goal_grid);
    a_start_graph.a_start_graph(graph);

    vector<V3F> path_points_prune;
    vector<V3F> path_points = a_start_graph.get_path_points();
    if (path_points.size() > 0)
    {
        a_start_graph.prune_path_by_collinearity(path_points, path_points_prune);
        send_waypoints(path_points_prune);
        for (int i = 0; i < path_points_prune.size(); i++) {
            path_points_prune[i].print();
            all_waypoints.push({ path_points_prune[i][0], path_points_prune[i][1], path_points_prune[i][2] });
        }
        flight_state = States::PLANNING;
    }
    else
    {
        in_planning = false;
    }
}

void ControlsFlyer::gps_sensor_callback()
{
    estimator->UpdateFromGPS(_posMeas, _velMeas);
    controller.UpdateEstimates(estimator->EstimatedPosition(), estimator->EstimatedVelocity(), estimator->EstimatedAttitude(), estimator->EstimatedOmega());
}

void ControlsFlyer::imu_sensor_callback()
{
    estimator->UpdateFromIMU(_accelMeas, _gyroMeas);
    estimator->UpdateFromMag(_magMeas[2]);
    clock_t osTick = clock();
    if (osTick >= nextPrediction)
    {
        float dt = (osTick - lastPrediction) / 1000;
        estimator->Predict(dt, _accelMeas, _gyroMeas);
        lastPrediction = osTick;
        nextPrediction = osTick + (1.0f / PREDICT_RATE) * 1000;
    }
    controller.UpdateEstimates(estimator->EstimatedPosition(), estimator->EstimatedVelocity(), estimator->EstimatedAttitude(), estimator->EstimatedOmega());
}

void ControlsFlyer::command_ack_callback()
{
    if (!m_bControlStatus && m_bTakeoffed)
    {
        cout << "cmd offboard on" << endl;
        cmd_position(local_position()[0], local_position()[1], -abs(target_position[2]), 0);
        getConnection()->make_command_flight_mode(FlightMode::Offboard);
        m_bControlStatus = true;
    }
}