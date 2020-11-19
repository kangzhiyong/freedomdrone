#include "motion_planning_flyer.hpp"

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

void MotionPlanning::find_closest_node(vector<point3D> nodes, point3D p, point3D& p_min)
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
    point3D goal = global_to_local({ -122.396428, 37.795128, target_altitude }, global_home());
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
