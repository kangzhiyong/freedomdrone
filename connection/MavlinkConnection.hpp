#pragma once

#include <ctime>
#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
using namespace std;

#include "MavSocket.hpp"
#include "MessageIDs.hpp"
#include "MessageTypes.hpp"
#include "MavUtils.hpp"
#include "Point.hpp"

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

 // bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF      0x1000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND         0x2000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER       0x3000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE         0x4000

class Drone;
/*
Implementation of the required communication to a drone executed
over the Mavlink protocol. Specifically designed with the PX4 autopilot in mind,
and currently been tested against that autopilot software.

Example:

    # TCP connection, protocol:ip:port
    conn = MavlinkConnection('tcp:127.0.0.1:5760')

    # Serial connection, port:baud
    conn = MavlinkConnection('5760:921600')
 */
class MavlinkConnection
{
public:
    typedef void (Drone::*notify_message_callback)(MessageIDs, void *);
    MavlinkConnection(std::string sock_type, std::string remote_ip, unsigned short remote_port, unsigned short local_port, bool threaded = false, bool PX4 = false, float send_rate = 5, time_t timeout = 5);
    
    void notify_message_listeners(MessageIDs name, void *msg);
    
    /*
     parse out the message based on the type and call
     the appropriate callbacks
     */
    void dispatch_message(mavlink_message_t *msg);
    
    bool open();
    void dispatch_loop();
    void dispatch_message(mavlink_message_t msg);
    void command_loop();
    bool wait_for_message(mavlink_message_t &msg);
    void start();
    void stop();
    void send_message(mavlink_message_t msg);
    void send_message_immediately(mavlink_message_t msg);
    void send(const vector<uint8_t>& packet);
    void send_long_command(uint16_t command_type, float param1, float param2=0, float param3=0, float param4=0, float param5=0, float param6=0, float param7=0);
    void arm();
    void disarm();
    void take_control();
    void release_control();
    void cmd_attitude_target_send(uint16_t type_mask, float roll, float pitch, float yaw, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust);
    void cmd_attitude(float roll, float pitch, float yaw, float thrust);
    void cmd_attitude_rate(float roll_rate, float pitch_rate, float yaw_rate, float thrust);
    void cmd_moment(float roll_moment, float pitch_moment, float yaw_moment, float thrust, time_t t=0.0);
    void msg_set_position_target_local_ned_pack(uint16_t mask, float n = 0, float e = 0, float d = 0, float vn = 0, float ve = 0, float vd = 0, float an = 0, float ae = 0, float ad = 0, float yaw=0, float yaw_rate=0, bool immediately=false);
    void cmd_velocity(float vn, float ve, float vd, float yaw);
    void cmd_position(float n, float e, float d, float yaw);
    void cmd_acceleration(float an, float ae, float ad, float yaw);
    void cmd_position(V4F p);
    void cmd_controls(float *controls, time_t t=0);
    void takeoff();
    void land(float n, float e);
    void set_home_position(float lat, float lon, float alt);
    void local_position_target(float n, float e, float d, time_t t=0);
    void local_velocity_target(float vn, float ve, float vd, time_t t=0);
    void local_acceleration_target(float an, float ae, float ad, time_t t=0);
    void attitude_target(float roll, float pitch, float yaw, time_t t=0);
    void body_rate_target(float p, float q, float r, time_t t=0);
    void set_sub_mode(int sub_mode);
    void set_notify_callback(notify_message_callback);
    void set_drone(Drone *drone)
    {
        _drone = drone;
    }
    MavSocket *getMaster()
    {
        return _master;
    }
    void cmd_offboard_control(bool flag);
    void handleCommandAck(mavlink_message_t& message);
    int reserveMavlinkChannel(void);
    void handleAttitudeTarget(mavlink_message_t& message);
    void make_command_flight_mode(FlightMode flight_mode);
    bool is_armed() const { return _armed; }
    LandedState landedState()
    {
        return _landState;
    }
    void msg_param_set(const char* param_id, float param_value);
private:
    MavSocket *_master;
    queue<mavlink_message_t> _out_msg_queue;
    thread *_read_handle;
    bool _read_handle_daemon{false};
    thread *_write_handle;
    bool _write_handle_daemon{false};
    // management
    bool _running{false};
    uint8_t _own_system{ 245 };
    uint8_t _own_component{ MAV_COMP_ID_MISSIONPLANNER };
    uint8_t _target_system{1};
    uint8_t _target_component{ MAV_COMP_ID_ALL };
    uint8_t _target_channel{ 1 };
    int autopilot_id{ MAV_COMP_ID_ALL };

    // PX4 management
    bool _using_px4{false};
    float _send_rate{5};
    bool writing_status{ false };

    // seconds to wait of no messages before termination
    time_t _timeout{5};
    mutex msg_queue_mutex;
    bool _threaded{false};
    Drone *_drone;
    notify_message_callback _notify_message_callback;
    bool            _globalPositionIntMessageAvailable{false};
    bool            _gpsRawIntMessageAvailable{false};
    uint32_t _mavlinkChannelsUsedBitMask;
    bool    _receivingAttitudeQuaternion{ false };
    std::atomic<bool> _armed{ false };
    std::atomic<bool> _hitl_enabled{ false };
    static constexpr double _ping_interval_s = 5.0;
    LandedState _landState{ LandedState::OnGround };
};
