//
//  message_ids.h
//  Drone
//
//  Created by kangzhiyong on 2020/2/25.
//

#ifndef message_ids_h
#define message_ids_h

enum message_ids
{
    ANY,
    STATE,
    GLOBAL_POSITION,
    LOCAL_POSITION,
    GLOBAL_HOME,
    LOCAL_VELOCITY,
    CONNECTION_CLOSED,
    RAW_GYROSCOPE,
    RAW_ACCELEROMETER,
    BAROMETER,
    DISTANCE_SENSOR,
    ATTITUDE,
    GPS_INPUT_SENSOR,
    RAW_IMU_SENSOR
};

#endif /* message_ids_h */
