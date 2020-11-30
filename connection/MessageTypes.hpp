/*
Message Types
custom set of message types to use between a specific connection type and
the drone class.
this enables abstracting away the protocol specific messages so different
protocols can be used with the same student facing interface and code.
NOTE: besides the state message, all messages are in the frame they are
defined in.
NOTE: to ensure minimal errors due to typos, use the MSG_* constants when
registering for specific messages
Attributes:
    MSG_ALL: flag to be used to register a listener for all messages
    MSG_STATE: name of the state message
    MSG_GLOBAL_POSITION: name of the global position message [StateMessage]
    MSG_LOCAL_POSITION: name of the local position message [LocalFrameMessage]
    MSG_GLOBAL_HOME: name of the global home message [GlobalFrameMessage]
    MSG_VELOCITY: name of the velocity message [LocalFrameMessage]
    MSG_CONNECTION_CLOSED: name of the message sent when the connection is closed (no data)
    MSG_RAW_GYROSCOPE: name of the raw gyro message [BodyFrameMessage]
    MSG_RAW_ACCELEROMETER: name of the raw acceleromater message [BodyFrameMessage]
    MSG_BAROMETER: name of the barometer message [LocalFrameMessage - only down populated]
    MSG_ATTITUDE: name of attitude message [FrameMessage]
*/

#pragma once

#include <cmath>
#include <vector>
using namespace std;

class MessageBase
{
    //Message super class
public:
    MessageBase(){_time = 0;}
    MessageBase(int64_t t);
    int64_t getTime();
private:
    int64_t _time;
};

class StateMessage: public MessageBase
{
    /*
     State information message
    message to carry drone state information
    Attributes:
        _armed: whether or not drone is armed
        _guided: whether or not drone can be commanded from script
    */

public:
    StateMessage(){};
    StateMessage(int64_t time, bool armed, bool guided, int status = 0);
    bool armed();
    bool guided();
    int status();
private:
    bool _armed{false};
    bool _guided{false};
    int _status{0};
};


class GlobalFrameMessage: public MessageBase
{
    /*
     Global frame message
    message to carry information in a global frame
    Attributes:
        _latitude: latitude in degrees
        _longitude: longitude in degrees
        _altitude: altitude in meters above mean sea level (AMSL)
    */
public:
    GlobalFrameMessage( int64_t time, float latitude, float longitude, float altitude);
    float longitude();
    float latitude();
    float altitude();
    vector<float> global_vector();
private:
    float _longitude;
    float _latitude;
    float _altitude;
};

class LocalFrameMessage: public MessageBase
{
    /*
     Local frame message
    message to carry information in a local (NED) frame
    Attributes:
        _north: north position in meters
        _east: east position in meters
        _down: down position in meters (above takeoff point, positive down)
    */
public:
    LocalFrameMessage( int64_t time, float north, float east, float down);
    float north();
    float east();
    float down();
    vector<float> local_vector();
private:
    float _north;
    float _east;
    float _down;
};

class BodyFrameMessage: public MessageBase
{
    /*
     Body frame message
    message to carry information in a body frame
    Attributes:
        _x: x value
        _y: y value
        _z: z value
    */
public:
    BodyFrameMessage(int64_t time, float x, float y, float z);
    float x();
    float y();
    float z();
    vector<float> body_vector();
private:
    float _x;
    float _y;
    float _z;
};

class FrameMessage: public MessageBase
{
    /*
        Message representating frame information
        Messages defining the rotation between frames (Euler angles or Quaternions)
        Attributes:
            _roll: drone roll in radians
            _pitch: drone pitch in radians
            _yaw: drone yaw in radians
            _q0: 0th element of quaterion
            _q1: 1th element of quaterion
            _q2: 2th element of quaterion
            _q3: 3th element of quaterion
    */
public:
    FrameMessage(int64_t time, float roll, float pitch, float yaw);
    FrameMessage(int64_t time, float q0, float q1, float q2, float q3);
    float roll();
    float pitch();
    float yaw();
    float q0();
    float q1();
    float q2();
    float q3();
    vector<float> euler_angles();
    vector<float> quaternions();
    
private:
    float _roll;
    float _pitch;
    float _yaw;
    float _q0;
    float _q1;
    float _q2;
    float _q3;
};

class DistanceSensorMessage: public MessageBase
{
    /*
    Message for distance sensor (e.g. Lidar) information
    the properties of and measurement from a given distance sensor onboard
    the drone.
    Attributes:
        _min_distance: minimum detectable distance in meters
        _max_distance: maximum detectable distance in meters
        _direction: the heading of the sensor for this measurement in radians
        _measurement: the distance measured in meters
        _covariance: the covariance of the measurement
    */
public:
    DistanceSensorMessage(int64_t time, float min_distance, float max_distance, float direction, float measurement, float covariance);
private:
    vector<float> measuremen;
    vector<float> properties;
};

class GPSSensorMessage : public MessageBase
{
    /*
    GPS sensor input message. This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system
    */
public:
    GPSSensorMessage(int64_t time, float lat, float lon, float alt, float vn, float ve, float vd);
    int32_t lat();
    int32_t lon();
    float alt();
    float vn();
    float ve();
    float vd();
private:
    int32_t _lat;   //Latitude (WGS84)
    int32_t _lon;   //Longitude (WGS84)
    float _alt;     //Altitude (MSL). Positive for up.
    float _vn;      //GPS velocity in north direction in earth-fixed NED frame
    float _ve;      //GPS velocity in east direction in earth-fixed NED frame
    float _vd;      //GPS velocity in down direction in earth-fixed NED frame
};

class RAWIMUSensorMessage: public MessageBase
{
    /*
    The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1).
    This message should always contain the true raw values without any scaling to allow data capture and system debugging.
    */
public:
    RAWIMUSensorMessage(int64_t time, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag): MessageBase(time)
    {
        _xacc = xacc;
        _yacc = yacc;
        _zacc = zacc;
        _xgyro = xgyro;
        _ygyro = ygyro;
        _zgyro = zgyro;
        _xmag = xmag;
        _ymag = ymag;
        _zmag = zmag;
    }
    int16_t xacc()
    {
        return _xacc;
    }
    int16_t yacc()
    {
        return _yacc;
    }
    int16_t zacc()
    {
        return _zacc;
    }
    int16_t xgyro()
    {
        return _xgyro;
    }
    int16_t ygyro()
    {
        return _xgyro;
    }
    int16_t zgyro()
    {
        return _xgyro;
    }
    int16_t xmag()
    {
        return _xmag;
    }
    int16_t ymag()
    {
        return _ymag;
    }
    int16_t zmag()
    {
        return _zmag;
    }
private:
    int16_t _xacc;  //X acceleration (raw)
    int16_t _yacc;  //Y acceleration (raw)
    int16_t _zacc;  //Z acceleration (raw)
    int16_t _xgyro; //Angular speed around X axis (raw)
    int16_t _ygyro; //Angular speed around Y axis (raw)
    int16_t _zgyro; //Angular speed around Z axis (raw)
    int16_t _xmag;  //X Magnetic field (raw)
    int16_t _ymag;  //Y Magnetic field (raw)
    int16_t _zmag;  //Z Magnetic field (raw)
};
