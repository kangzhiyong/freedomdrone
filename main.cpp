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

    FreeData<double> data("../../../data/colliders.csv", ",");
    vector<int> grid;
    int nrows, ncols = 0;
    int drone_alt = 10, safe_distance = 3;
    data.createGrid(drone_alt, safe_distance, grid, nrows, ncols);
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

    /*int ncols = 500, nrows = 300;
    std::vector<float> z(ncols * nrows);
    for (int j = 0; j < nrows; ++j) {
        for (int i = 0; i < ncols; ++i) {
            z.at(ncols * j + i) = std::sin(std::hypot(i - ncols / 2, j - nrows / 2));
        }
    }

    const float* zptr = &(z[0]);*/

    const int colors = 1;

    plt::imshow(zptr, nrows, ncols, colors);
    plt::xlabel("EAST");
    plt::ylabel("NORTH");
    plt::show();
    //// Show plots
    //plt::save("imshow.png");
    //std::cout << "Result saved to 'imshow.png'.\n";


    //plt::plot({1,3,2,4});
    //plt::show();

    //int n = 5000;
    //std::vector<double> x(n), y(n), z(n), w(n, 2);
    //for (int i = 0; i < n; ++i) {
    //    x.at(i) = i * i;
    //    y.at(i) = sin(2 * M_PI * i / 360.0);
    //    z.at(i) = log(i);
    //}

    //// Set the size of output image to 1200x780 pixels
    //plt::figure_size(1200, 780);
    //// Plot line from given x and y data. Color is selected automatically.
    //plt::plot(x, y);
    //// Plot a red dashed line from given x and y data.
    //plt::plot(x, w, "r--");
    //// Plot a line whose name will show up as "log(x)" in the legend.
    //plt::named_plot("log(x)", x, z);
    //// Set x-axis to interval [0,1000000]
    //plt::xlim(0, 1000 * 1000);
    //// Add graph title
    //plt::title("Sample figure");
    //// Enable legend.
    //plt::legend();
    //// Save the image (file format is determined by the extension)
    //plt::save("./basic.png");

    return 0;
}
