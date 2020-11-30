//
//  main.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/4.
//

#include <stdio.h>

#include "ControlsFlyer.hpp"

int main()
{
    MavlinkConnection conn("TCP", "127.0.0.1", 5760, false, false);
    ControlsFlyer drone(&conn);
    drone.start_drone();
    drone.print_mission_score();
    return 0;
}
