//
//  main.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/4.
//

#include <stdio.h>

#include <msgpack.hpp>
#include <string>
#include <iostream>
#include <sstream>

#include "freedomdrone.h"
#include "search_algorithm.hpp"
#include "free_utils.hpp"
#include "free_data.hpp"
#include "matplotlibcpp.h"
#include "free_graph.hpp"

namespace plt = matplotlibcpp;

void test_wikipedia() {
    typedef point<int, 2> point2d;
    typedef FreeKDTree<int, 2> tree2d;

    point2d points[] = { { 2, 3 }, { 5, 4 }, { 9, 6 }, { 4, 7 }, { 8, 1 }, { 7, 2 } };

    tree2d tree(std::begin(points), std::end(points));
    point2d n = tree.nearest({ 9, 2 });

    std::cout << "Wikipedia example data:\n";
    std::cout << "nearest point: " << n << '\n';
    std::cout << "distance: " << tree.distance() << '\n';
    std::cout << "nodes visited: " << tree.visited() << '\n';
}

typedef point<double, 3> point3d;
typedef FreeKDTree<double, 3> tree3d;

struct random_point_generator {
    random_point_generator(double min, double max)
        : engine_(std::random_device()()), distribution_(min, max) {}

    point3d operator()() {
        double x = distribution_(engine_);
        double y = distribution_(engine_);
        double z = distribution_(engine_);
        return point3d({ x, y, z });
    }

    std::mt19937 engine_;
    std::uniform_real_distribution<double> distribution_;
};

void test_random(size_t count) {
    random_point_generator rpg(0, 1);
    tree3d tree(rpg, count);
    point3d pt(rpg());
    point3d n = tree.nearest(pt);

    std::cout << "Random data (" << count << " points):\n";
    std::cout << "point: " << pt << '\n';
    std::cout << "nearest point: " << n << '\n';
    std::cout << "distance: " << tree.distance() << '\n';
    std::cout << "nodes visited: " << tree.visited() << '\n';
}

int main()
{
//    msgpack::type::tuple<float, float, float> src(111, 222, 333);
//
//    // serialize the object into the buffer.
//    // any classes that implements write(const char*,size_t) can be a buffer.
//    std::stringstream buffer;
//    msgpack::pack(buffer, src);
//    cout << buffer.str() << endl;
//    // send the buffer ...
//    buffer.seekg(0);
//
//    // deserialize the buffer into msgpack::object instance.
//    std::string str(buffer.str());

//    msgpack::object_handle oh =
//        msgpack::unpack(str.data(), str.size());
//
//    // deserialized object is valid during the msgpack::object_handle instance is alive.
//    msgpack::object deserialized = oh.get();

    // msgpack::object supports ostream.
//    std::cout << deserialized << std::endl;

    // convert msgpack::object instance into the original type.
    // if the type is mismatched, it throws msgpack::type_error exception.
//    msgpack::type::tuple<int, bool, std::string> dst;
//    deserialized.convert(dst);
//
//    // or create the new instance
//    msgpack::type::tuple<int, bool, std::string> dst2 =
//        deserialized.as<msgpack::type::tuple<int, bool, std::string> >();
    //try {
    //    test_wikipedia();
    //    std::cout << '\n';
    //    test_random(1000);
    //    std::cout << '\n';
    //    test_random(1000000);
    //}
    //catch (const std::exception& e) {
    //    std::cerr << e.what() << '\n';
    //}
    //priority_queue<GirdCellCoord> q;
    //q.push(GirdCellCoord(1, 2, 13));
    //q.push(GirdCellCoord(1, 2, 4));
    //q.push(GirdCellCoord(1, 2, 1));
    //q.push(GirdCellCoord(1, 2, 6));
    //q.push(GirdCellCoord(1, 2, 10));
    //cout << q.size() << endl;
    //int n = q.size();
    //while(!q.empty())
    //{
    //    cout << q.top().queue_cost << endl;
    //    q.pop();
    //}
    //return 0;
//    MavlinkConnection conn("TCP", "127.0.0.1", 5760, false, false);
//    BackyardFlyer drone(&conn);
//    drone.start_drone();
    
    /*SearchAlgorithm bfs;
    bfs.breadth_first();
    bfs.visualize_path();

    SearchAlgorithm dfs;
    dfs.depth_first();
    dfs.visualize_path();

    SearchAlgorithm astar;
    astar.a_star();
    astar.visualize_path();

    point3DD gc({ -122.079465, 37.393037, 30 });
    point3DD gh({ -122.108432, 37.400154, 20 });
    point3DD lNEDS({ 25.21, 128.07, -30. });
    point3DD lNED = global_to_local(gc, gh);
    lNED.print("LNED: ");

    gc = local_to_global(lNEDS, gh);
    gc.print();  */
    
//    string path = "../../data/colliders.csv";
//#ifdef WIN32
//    path = "../../../data/colliders.csv";
//#endif
//    FreeData<float> data(path, ",");
//    vector<float> grid;
//    float drone_altitude = 5, safe_distance = 0;
//    data.createGrid(drone_altitude, safe_distance, grid, g_north_size, g_east_size, g_alt_size);
//
//    const float* zptr = (float *)&(grid[0]);
//
//    // For the purposes of the visual the east coordinate lay along
//    // the x-axis and the north coordinates long the y-axis.
//    int colors = 1;
    //GirdCellType start_ne(25, 100), goal_ne(750, 370);
//    SearchAlgorithm astar(start_ne, goal_ne, grid);
//    astar.a_star();
//    vector< point<int, 2> > path_points = astar.get_path_points();
//    
//    plt::imshow(zptr, g_north_size, g_east_size, colors, {{"cmap", "Greys"}, {"origin", "lower"}});
//    vector< point<int, 2>> path_points_prune;
////    prune_path_by_collinearity(path_points, path_points_prune);
//    astar.prune_path_by_bresenham(path_points, path_points_prune);
//    plt::plot({(double)start_ne.getY()}, {(double)start_ne.getX()}, "X");
//    plt::plot({(double)goal_ne.getY()}, {(double)goal_ne.getX()}, "X");
//    
//    vector<int> pp_x, pp_y;
//    for (size_t i = 0; i < path_points_prune.size(); i++) {
//        pp_x.push_back(path_points_prune[i][0]);
//        pp_y.push_back(path_points_prune[i][1]);
//    }
//    plt::scatter(pp_y, pp_x);
//    plt::plot(pp_y, pp_x, "deeppink");
//    plt::xlabel("EAST");
//    plt::ylabel("NORTH");
//    plt::show();
    
//    vector<int> voxmap;
//    data.create_voxmap(10, voxmap);
//
//    const float* zptr = (float *)&(voxmap[0]);
//    try
//    {
//        plt::voxels(zptr, g_north_size, g_east_size, g_alt_size, { { "edgecolor" , "k" } });
//        plt::xlim(g_north_size, 0);
//        plt::ylim(0, g_east_size);
//        //plt::zlim(0, g_alt_size + 20);
//        plt::xlabel("North");
//        plt::ylabel("East");
//        plt::legend();
//        plt::show();
//    }
//    catch (const std::exception& e)
//    {
//        cout << e.what() << endl;
//    }

//    data.extract_polygons(safe_distance);
//    data.sample(500);
//
//    data.create_graph(10);
//
//    FreeGraph<float, 3> graph = data.getGraph();
//    vector<FreeEdge<float, 3>> edges;
//    vector<point3D> nodes;
//    graph.getAllNodesAndEdges(nodes, edges);
//
//    cout << nodes.size() << " " << edges.size() << endl;
//    GirdCellType start_ne(nodes[0]), goal_ne(nodes[nodes.size() - 1]);
//    SearchAlgorithm a_start_graph(start_ne, goal_ne);
//    a_start_graph.a_start_graph(graph);
//    plt::plot({(double)start_ne.get(1)}, {(double)start_ne.get(0)}, "X");
//    plt::plot({(double)goal_ne.get(1)}, {(double)goal_ne.get(0)}, "X");
//    vector< point3D > path_points = a_start_graph.get_path_points();
//
//    plt::imshow(zptr, g_north_size, g_east_size, colors, { {"cmap", "Greys"}, {"origin", "lower"} });
//
//    vector<float> pp_x, pp_y, pp_z;
//    FreeEdge<float, 3> edge;
//    for (int i = 0; i < edges.size(); i++) {
//        edge = edges[i];
//        pp_x.push_back(edge.getStart()[0]);
//        pp_x.push_back(edge.getEnd()[0]);
//        pp_y.push_back(edge.getStart()[1]);
//        pp_y.push_back(edge.getEnd()[1]);
//        plt::plot(pp_y, pp_x, "g");
//        pp_x.clear();
//        pp_y.clear();
//    }
//
//
//    vector<point3D> allNodes = data.getSamplePoints();
//    for (size_t i = 0; i < allNodes.size(); i++) {
//        pp_x.push_back(allNodes[i][0]);
//        pp_y.push_back(allNodes[i][1]);
//        plt::scatter(pp_y, pp_x, 30, { {"c", "black"} });
//        pp_x.clear();
//        pp_y.clear();
//    }
//
//    pp_x.clear();
//    pp_y.clear();
//    for (size_t i = 0; i < nodes.size(); i++) {
//        pp_x.push_back(nodes[i][0]);
//        pp_y.push_back(nodes[i][1]);
//        plt::scatter(pp_y, pp_x, 30, { {"c", "black"} });
//        pp_x.clear();
//        pp_y.clear();
//    }
//
//    pp_x.clear();
//    pp_y.clear();
//    for (size_t i = 0; i < path_points.size(); i++) {
//        pp_x.push_back(path_points[i][0]);
//        pp_y.push_back(path_points[i][1]);
//    }
//    plt::scatter(pp_y, pp_x, 30, { {"c", "pink"} });
//    plt::plot(pp_y, pp_x, "r");
//
//    plt::ylabel("EAST");
//    plt::xlabel("NORTH");
//    plt::show();
    
    MavlinkConnection conn("TCP", "127.0.0.1", 5760, false, false);
    MotionPlanning drone(&conn);
    drone.start_drone();
    return 0;
}
