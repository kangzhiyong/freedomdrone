// freedomdrone.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once

#include <iostream>
using namespace std;

#include "drone.hpp"
#include "mavlink_connection.hpp"
#include "free_point.hpp"
#include "free_data.hpp"
#include "msgpack.hpp"

/*
# NOTE: a waypoint here is defined as[North, East, Down]

###### EXAMPLES ######
#
# here are a set of example waypoint sets that you might find useful for testing.
# each has a bit of a description to help with the potential use case for the waypoint set.
#
# NOTE: the waypoint lists are defined as a list of lists, which each entry in a list of the
# [North, East, Down] to command.  Also recall for the crazyflie, North and East are defined
# by the starting position of the drone(straight and left, respectively), not world frame North
#and East.
#
###### ######## ######

######
# 1. have the crazyflie hover in a single place.
#
# For this to work best, make sure to comment out the waypoint transition code(see block comment
# in `local_position_callback`) to ensure that the crazyflie attempts to hold this position.
######

WAYPOINT_LIST = [[0.0, 0.0, -0.5]]


######
# 2. there and back.
#
# Simple 2 point waypoint path to go away and come back.
######

# WAYPOINT_LIST = [
#     [1.5, 0.0, -0.5],
#     [0.0, 0.0, -0.5]
#     ]


######
# 3. simple box.
#
# A simple box, much like what you flew for the backyard flyer examples.
######

                # WAYPOINT_LIST = [
#     [1.0, 0.0, -0.5],
#     [1.0, 1.0, -0.5],
#     [0.0, 1.0, -0.5],
#     [0.0, 0.0, -0.5]
#     ]
*/

// that height to which the drone should take off
#define TAKEOFF_ALTITUDE 0.5

typedef void (*CallBackFunc)(void *userData, double lo, double la, double alt);

enum States
{
    MANUAL = 0,
    ARMING,
    TAKEOFF,
    WAYPOINT,
    LANDING,
    DISARMING,
    PLANNING
};

class MSGPoint
{
private:
    int x;
    int y;
    int z;
    int a;
public:
    MSGPoint(point3D p)
    {
        x = p[0];
        y = p[1];
        z = p[2];
        a = 0;
    }
    MSGPACK_DEFINE(x, y, z, a);
};