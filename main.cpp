//
//  main.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/4.
//

#include <stdio.h>

#include "VelocityFlyer.hpp"

int main()
{
    MavlinkConnection conn("UDP", "192.168.137.128", REMOTE_PORT_ONBOARD, LOCAL_PORT_ONBOARD, true, true);
    VelocityFlyer drone(&conn);
    drone.start_drone();
    return 0;
}
