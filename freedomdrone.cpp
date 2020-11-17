// freedomdrone.cpp: 定义应用程序的入口点。
//

#include <msgpack.hpp>
#include <string>
#include <iostream>
#include <sstream>

#include "freedomdrone.h"
#include "free_utils.hpp"
#include "search_algorithm.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

//#include <msgpack.hpp>

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
        cout << "takeoff locap, targetp" << endl;
        local_position().print();
        target_position.print();
        cout << endl;

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
    msgpack::sbuffer sbuf;
    std::stringstream buffer;
    msgpack::packer<msgpack::sbuffer> pk(sbuf);
    for (size_t i = 0; i < points.size(); i++)
    {
        pk.pack(std::tuple <float, float, float>(points[i][0], points[i][1], points[i][2]));
        msgpack::pack(buffer, std::tuple <float, float, float>(points[i][0], points[i][1], points[i][2]));
    }
    cout.setf(ios::showbase);
    cout.setf(ios_base::hex, ios_base::basefield);
    cout << sbuf.data() << endl;
    cout << buffer.str() << endl;
    cout.unsetf(ios::hex);

    //getConnection()->getMaster()->write((void*)sbuf.data(), sbuf.size());
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
    point3D home_ne({125, 245, 0});
    point3D home_glo = local_to_global(home_ne, global_home());
    home_glo.print();
    cmd_position(home_ne, 0);
    //_update_local_position(global_to_local(global_position(), global_home()));
    
    data.extract_polygons(safety_distance);
    data.sample(1000);
    data.create_graph(10);
    point3D start = local_position();
    point3D goal = point3D({ 230, 380, 20 })/*global_to_local({-122.396428, 37.795128, target_altitude}, global_home())*/;
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
    vector<float> grid;
    data.createGrid(target_altitude, safety_distance, grid, g_north_size, g_east_size, g_alt_size);
    
    GirdCellType start_grid(start_new);
    GirdCellType goal_grid(goal_new);
    SearchAlgorithm a_start_graph(start_grid, goal_grid, grid);
    a_start_graph.a_start_graph(graph);
    
    vector<point3D> path_points_prune;
    vector<point3D> path_points = a_start_graph.get_path_points();
    if (path_points.size() > 0)
    {
        a_start_graph.prune_path_by_collinearity(path_points, path_points_prune);
        send_waypoints(path_points_prune);
        for (int i = 0; i < path_points_prune.size(); i++) {
            all_waypoints.push({ path_points_prune[i][0], path_points_prune[i][1], path_points_prune[i][2] });
        }
        flight_state = PLANNING;
    }

    //const float* zptr = (float *)&(grid[0]);
    //int colors = 1;
    //plt::plot({(double)start[1]}, {(double)start[0]}, "bX");
    //plt::plot({(double)goal[1]}, {(double)goal[0]}, "cX");
    //plt::plot({(double)start_new[1]}, {(double)start_new[0]}, "bX");
    //plt::plot({(double)goal_new[1]}, {(double)goal_new[0]}, "cX");
    //plt::imshow(zptr, g_north_size, g_east_size, colors, { {"cmap", "Greys"}, {"origin", "lower"} });

    //vector<float> pp_x, pp_y, pp_z;
    //FreeEdge<float, 3> edge;
    //for (int i = 0; i < edges.size(); i++) {
    //    edge = edges[i];
    //    pp_x.push_back(edge.getStart()[0]);
    //    pp_x.push_back(edge.getEnd()[0]);
    //    pp_y.push_back(edge.getStart()[1]);
    //    pp_y.push_back(edge.getEnd()[1]);
    //    plt::plot(pp_y, pp_x, "g");
    //    pp_x.clear();
    //    pp_y.clear();
    //}


    ////vector<point3D> allNodes = data.getSamplePoints();
    ////for (size_t i = 0; i < allNodes.size(); i++) {
    ////    pp_x.push_back(allNodes[i][0]);
    ////    pp_y.push_back(allNodes[i][1]);
    ////    plt::scatter(pp_y, pp_x, 30, { {"c", "black"} });
    ////    pp_x.clear();
    ////    pp_y.clear();
    ////}

    //pp_x.clear();
    //pp_y.clear();
    //for (size_t i = 0; i < nodes.size(); i++) {
    //    pp_x.push_back(nodes[i][0]);
    //    pp_y.push_back(nodes[i][1]);
    //    plt::scatter(pp_y, pp_x, 30, { {"c", "red"} });
    //    pp_x.clear();
    //    pp_y.clear();
    //}

    //pp_x.clear();
    //pp_y.clear();
    //for (size_t i = 0; i < path_points_prune.size(); i++) {
    //    pp_x.push_back(path_points_prune[i][0]);
    //    pp_y.push_back(path_points_prune[i][1]);
    //}
    //plt::scatter(pp_y, pp_x, 30, { {"c", "pink"} });
    //plt::plot(pp_y, pp_x, "r");

    //plt::ylabel("EAST");
    //plt::xlabel("NORTH");
    //plt::show();
}
