//
//  main.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/4.
//

#include <stdio.h>
#include "freedomdrone.h"
#include "search_algorithm.hpp"
#include "free_utils.hpp"
#include "free_data.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main()
{
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
    
    string path = "../../data/colliders.csv";
#ifdef WIN32
    path = "../../../data/colliders.csv";
#endif
    FreeData<double> data(path, ",");
//    vector<int> grid;
//    int drone_altitude = 5, safe_distance = 3;
//    data.createGrid(drone_altitude, safe_distance, grid);
//
//    const float* zptr = (float *)&(grid[0]);
//
//    // For the purposes of the visual the east coordinate lay along
//    // the x-axis and the north coordinates long the y-axis.
//    int colors = 1;    
//    GirdCellCoord start_ne(25, 100), goal_ne(750, 370);
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
    
    vector<int> voxmap;
    data.create_voxmap(10, voxmap);

    const float* zptr = (float *)&(voxmap[0]);
    try
    {
        plt::voxels(zptr, g_north_size, g_east_size, g_alt_size, { { "edgecolor" , "k" } });
        plt::xlim(g_north_size, 0);
        plt::ylim(0, g_east_size);
        //plt::zlim(0, g_alt_size + 20);
        plt::xlabel("North");
        plt::ylabel("East");
        plt::legend();
        plt::show();
    }
    catch (const std::exception& e)
    {
        cout << e.what() << endl;
    }

    return 0;
}
