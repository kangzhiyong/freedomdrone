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
    MavlinkConnection conn("UDP", "192.168.137.130", REMOTE_PORT_ONBOARD, LOCAL_PORT_ONBOARD, true, true);
    ControlsFlyer drone(&conn);
    drone.start_drone();
    return 0;
}
