#include "MotionPlanning.hpp"
#include "SearchAlgorithm.hpp"

MotionPlanning::MotionPlanning(MavlinkConnection* conn) : Drone(conn)
{
    // register all your callbacks here
    register_callback(MessageIDs::LOCAL_POSITION, ((void (Drone::*)()) & MotionPlanning::local_position_callback));
    register_callback(MessageIDs::LOCAL_VELOCITY, ((void (Drone::*)()) & MotionPlanning::velocity_callback));
    register_callback(MessageIDs::STATE, ((void (Drone::*)()) & MotionPlanning::state_callback));
    register_callback(MessageIDs::COMMANDACKRESULT, ((void (Drone::*)()) & MotionPlanning::command_ack_callback));
}

void MotionPlanning::local_position_callback()
{
    if (flight_state == States::TAKEOFF)
    {
        if (abs(abs(local_position()[2]) - abs(target_position[2])) < 0.1)
        {
            waypoint_transition();
        }
    }
    else if (flight_state == States::WAYPOINT)
    {
        if ((target_position - local_position()).mag() < 1.0)
        {
            if (all_waypoints.size() > 0)
            {
                waypoint_transition();
            }
            else if (local_velocity().mag() < 1.0)
            {
                landing_transition();
            }
        }
    }
}

void MotionPlanning::velocity_callback()
{
    if (flight_state == States::LANDING)
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
        if (flight_state == States::MANUAL)
        {
            if (!in_planning)
            {
                new thread(&MotionPlanning::plan_path, this);
                in_planning = true;
            }
        }
        else if (flight_state == States::ARMING && armed())
        {
            takeoff_transition();
        }
        else if (flight_state == States::PLANNING)
        {
            arming_transition();
        }
        else if (flight_state == States::DISARMING && !armed() && !guided())
        {
            manual_transition();
        }
    }
}

void MotionPlanning::arming_transition()
{
    flight_state = States::ARMING;
    cout << "arming transition\r\n" << endl;
    arm();
}

void MotionPlanning::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    local_position().print();
    float target_altitude = local_position()[2] + 10.0;
    target_position[2] = target_altitude;
    takeoff(/*target_position[2]*/);
    flight_state = States::TAKEOFF;
}

void MotionPlanning::waypoint_transition()
{
    flight_state = States::WAYPOINT;
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
    flight_state = States::LANDING;
    cout << "landing transition" << endl;
    land();
}

void MotionPlanning::disarming_transition()
{
    flight_state = States::DISARMING;
    cout << "disarm transition" << endl;
    disarm();
}

void MotionPlanning::manual_transition()
{
    flight_state = States::MANUAL;
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
}

void MotionPlanning::start_drone()
{
    cout << "starting connection" << endl;
    start();
}

void MotionPlanning::find_closest_node(vector<V3F> nodes, V3F p, V3F& p_min)
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

void MotionPlanning::send_waypoints(vector<V3F> points)
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
    //set_home_position(lon0, lat0, 0);
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
    cout << "star graph end" << endl;
    vector<V3F> path_points_prune;
    vector<V3F> path_points = a_start_graph.get_path_points();
    cout << "get path points end" << path_points.size() << endl;
    if (path_points.size() > 0)
    {
        a_start_graph.prune_path_by_collinearity(path_points, path_points_prune);
        cout << "prune end" << endl;
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
    //V3F cp = local_position();
    //all_waypoints.push(cp + V3F({ 10.0, 0.0, 10 }));
    //all_waypoints.push(cp + V3F({ 10.0, 10.0, 10 }));
    //all_waypoints.push(cp + V3F({ 0.0, 10.0, 10 }));
    //all_waypoints.push(cp + V3F({ 0.0, 0.0, 10 }));
    //flight_state = States::PLANNING;
}

void MotionPlanning::command_ack_callback()
{
    //if (!m_bControlStatus && m_bTakeoffed)
    //{
    //    cout << "cmd offboard on" << endl;
    //    cmd_position(local_position()[0], local_position()[1], -abs(target_position[2]), 0);
    //    getConnection()->make_command_flight_mode(FlightMode::Offboard);
    //    m_bControlStatus = true;
    //}
}