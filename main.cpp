//
//  main.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/4.
//

#include <stdio.h>
#include "freedomdrone.h"
#include "search_algorithm.hpp"

int main()
{
//    MavlinkConnection conn("TCP", "127.0.0.1", 5760, false, false);
//    BackyardFlyer drone(&conn);
//    drone.start_drone();
    
    SearchAlgorithm bfs;
    bfs.breadth_first();
    bfs.visualize_path();

    SearchAlgorithm dfs;
    dfs.depth_first();
    dfs.visualize_path();

    SearchAlgorithm astar;
    astar.a_star();
    astar.visualize_path();
    return 0;
}
