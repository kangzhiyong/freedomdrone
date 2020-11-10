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
    vector<int> grid;
    int nrows, ncols = 0;
    int drone_altitude = 5, safe_distance = 3;
    data.createGrid(drone_altitude, safe_distance, grid, nrows, ncols);
    vector<float> z(ncols * nrows);
    for (size_t i = 0; i < ncols; i++)
    {
        for (size_t j = 0; j < nrows; j++)
        {
            z[nrows * (ncols - i - 1) + j] = grid[nrows * i + j];
        }
        cout << endl;
    }
    const float* zptr = &(z[0]);

    int colors = 1;
//
//    plt::imshow(zptr, nrows, ncols, colors);
//    plt::xlabel("EAST");
//    plt::ylabel("NORTH");
//    plt::show();

    // For the purposes of the visual the east coordinate lay along
    // the x-axis and the north coordinates long the y-axis.
    plt::imshow(zptr, nrows, ncols, colors);
    GirdCellCoord start_ne(25, 100), goal_ne(750, 370);
//    vector<int> ne_x, ne_y;
//    ne_x.push_back(start_ne.getX());
//    ne_x.push_back(goal_ne.getX());
//    ne_y.push_back(start_ne.getY());
//    ne_y.push_back(goal_ne.getY());
    
//    plt::plot(start_ne.getY(), start_ne.getX(), "x");
//    plt::plot(goal_ne.getY(), goal_ne.getX(), "x");
    
    SearchAlgorithm astar(start_ne, goal_ne);
    astar.a_star();
    vector< point<int, 2> > path_points = astar.get_path_points();
    vector<int> pp_x, pp_y;
    for (size_t i = 0; i < path_points.size(); i++) {
        pp_x.push_back(path_points[i][0]);
        pp_y.push_back(path_points[i][1]);
    }
    plt::plot(pp_x, pp_y, "g");
    plt::xlabel("EAST");
    plt::ylabel("NORTH");
    plt::show();
    
    plt::imshow(zptr, nrows, ncols, colors);
    vector< point<int, 2>> path_points_prune;
    prune_path_by_collinearity(path_points, path_points_prune);
    
//    plt::plot(start_ne.getY(), start_ne.getX(), "x");
//    plt::plot(goal_ne.getY(), goal_ne.getX(), "x");
    
    pp_x.clear();
    pp_y.clear();
    for (size_t i = 0; i < path_points_prune.size(); i++) {
        pp_x.push_back(path_points_prune[i][0]);
        pp_y.push_back(path_points_prune[i][1]);
    }
    plt::plot(pp_x, pp_y, "g");
    plt::xlabel("EAST");
    plt::ylabel("NORTH");
    plt::show();
    
//    point2D p0({0, 0}), p1({7, 5});
//    vector<float> line_x({0, 7});
//    vector<float> line_y({0, 5});
//    vector<point2D> cells;
//    bresenham({line_x[0], line_y[0]}, {line_x[1], line_y[1]}, cells);
//    plt::plot(line_x, line_y);
//
//    for (size_t i = 0; i < cells.size(); i++) {
//        point2D p = cells[i];
//        line_x.clear();
//        line_y.clear();
//        line_x.push_back(p[0]);
//        line_x.push_back(p[0] + 1);
//        line_y.push_back(p[1]);
//        line_y.push_back(p[1]);
//        plt::plot(line_x, line_y, "k");
//
//        line_x.clear();
//        line_y.clear();
//        line_x.push_back(p[0]);
//        line_x.push_back(p[0] + 1);
//        line_y.push_back(p[1] + 1);
//        line_y.push_back(p[1] + 1);
//        plt::plot(line_x, line_y, "k");
//
//        line_x.clear();
//        line_y.clear();
//        line_x.push_back(p[0]);
//        line_x.push_back(p[0]);
//        line_y.push_back(p[1]);
//        line_y.push_back(p[1] + 1);
//        plt::plot(line_x, line_y, "k");
//
//        line_x.clear();
//        line_y.clear();
//        line_x.push_back(p[0] + 1);
//        line_x.push_back(p[0] + 1);
//        line_y.push_back(p[1]);
//        line_y.push_back(p[1] + 1);
//        plt::plot(line_x, line_y, "k");
//    }
//
//    plt::grid(true);
//    plt::axis("equal");
//    plt::xlabel("X");
//    plt::ylabel("Y");
//    plt::title("Python package Bresenham algorithm");
//    plt::show();
    
    return 0;
}
