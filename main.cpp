//
//  main.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/4.
//

#include <stdio.h>
#include "freedomdrone.h"

int main()
{
    MavlinkConnection conn("TCP", "127.0.0.1", 5760, false, false);
    BackyardFlyer drone(&conn);
    drone.start_drone();
    return 0;
}
