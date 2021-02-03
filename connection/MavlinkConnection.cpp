//
//  mavlink_connection.cpp
//  Drone
//
//  Created by kangzhiyong on 2020/2/26.
//
#include "mavlink/common/common.hpp"
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink/common/mavlink.h"
#include "mavlink/ardupilotmega/ardupilotmega.h"

#include "MavlinkConnection.hpp"
//#include "Utility/StringUtils.h"
#include "MavUtils.hpp"
#include "DroneUtils.hpp"

MavlinkConnection::MavlinkConnection(std::string sock_type, std::string remote_ip, unsigned short remote_port, unsigned short local_port, bool threaded, bool PX4, float send_rate, time_t timeout)
{
    /*
    Constructor for Mavlink based drone connection.
    Note: When threaded, the read loop runs as a daemon, meaning once all
    other processes stop the thread immediately dies, therefore some
    acitivty (e.g. a while True loop) needs to be running on the main
    thread for this thread to survive.

    Args:
        device: address to the drone, e.g. "tcp:127.0.0.1:5760" (see mavutil mavlink connection for valid options)
        threaded: bool for whether or not to run the message read loop on a separate thread
        PX4: bool for whether or not connected to a PX4 autopilot. Determines the behavior of the
            command loop (write thread)
        send_rate: the rate in Hertz (Hz) to send messages to the drone
        timeout: how long to wait for a message before the connection times out (seconds)
    */

    // create the connection
    if (sock_type == "TCP")
    {
        _master = new MavTCP(remote_ip, remote_port, local_port);
    }
    else
    {
        _master = new MavUDP(remote_ip, remote_port, local_port);
    }

    int mavlinkChannel = reserveMavlinkChannel();
    if (mavlinkChannel != 0) {
        _target_channel = mavlinkChannel;
    }
    else {
        cout << "Ran out of mavlink channels" << endl;
        return;
    }

    _threaded = threaded;

    // PX4 management
    _using_px4 = PX4;
    _send_rate = send_rate;

    // seconds to wait of no messages before termination
    _timeout = timeout;
}

int MavlinkConnection::reserveMavlinkChannel(void)
{
    // Find a mavlink channel to use for this link, Channel 0 is reserved for internal use.
    for (uint8_t mavlinkChannel = 1; mavlinkChannel < MAVLINK_COMM_NUM_BUFFERS; mavlinkChannel++) {
        if (!(_mavlinkChannelsUsedBitMask & 1 << mavlinkChannel)) {
            mavlink_reset_channel_status(mavlinkChannel);
            // Start the channel on Mav 1 protocol
            mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(mavlinkChannel);
            mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            _mavlinkChannelsUsedBitMask |= 1 << mavlinkChannel;
            return mavlinkChannel;
        }
    }
    return 0;   // All channels reserved
}

bool MavlinkConnection::open()
{
    return (_master->get_socket_fd() == -1)? false : true;
}

void MavlinkConnection::dispatch_loop()
{
    /*
    Main loop to read from the drone.

    Continually listens to the drone connection for incoming messages.
    for each new message, parses out the mavlink, creates messages as
    defined in `message_types.py`, and triggers all callbacks registered
    for that type of message.
    Also keeps an eye on the state of connection, and if nothing has
    happened in more than 5 seconds, sends a special termination message
    to indicate that the drone connection has died.

    THIS SHOULD NOT BE CALLED DIRECTLY BY AN OUTSIDE CLASS!
    */
    bool valid_msg = false;
    time_t last_msg_time = time(nullptr);
    while(_running)
    {
        // wait for a new message
        mavlink_message_t msg;
        valid_msg = wait_for_message(msg);

        // if no message or a bad message was received, just move along
        if (!valid_msg)
            continue;

        time_t current_time = time(nullptr);

        // print("Time between messages", current_time - last_msg_time)

        // if we haven't heard a message in a given amount of time
        // send a termination message
//        if (current_time - last_msg_time > _timeout)
//        {
//            MessageBase data;
//            // notify listeners that the connection is closing
//            notify_message_listeners(CONNECTION_CLOSED, data);
//
//            // stop this read loop
//            _running = false;
//        }
        // update the time of the last message
        last_msg_time = current_time;

        dispatch_message(msg);
        // give the write thread time to use the port
        if (writing_status > false) {
            usleep(100); // look for components of batches at 10kHz
        }
    }
}

/*
 parse out the message based on the type and call
 the appropriate callbacks
 */
void MavlinkConnection::dispatch_message(mavlink_message_t msg)
{
	//_target_system = msg.sysid;
	//autopilot_id = msg.compid;
    //https://mavlink.io/en/messages/common.html#messages
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            mavlink_global_position_int_t gpi_msg;
            memset(&gpi_msg, 0, sizeof(mavlink_global_position_int_t));
            mavlink_msg_global_position_int_decode(&msg, &gpi_msg);

            // ArduPilot sends bogus GLOBAL_POSITION_INT messages with lat/lat 0/0 even when it has no gps signal
            // Apparently, this is in order to transport relative altitude information.
            if (gpi_msg.lat == 0 && gpi_msg.lon == 0) {
                return;
            }

            _globalPositionIntMessageAvailable = true;
            
            uint32_t timestamp = gpi_msg.time_boot_ms / 1000.0;
            // parse out the gps position and trigger that callback
            GlobalFrameMessage gps(timestamp, float(gpi_msg.lat) / 1e7, float(gpi_msg.lon) / 1e7, float(gpi_msg.alt) / 1000);
            notify_message_listeners(MessageIDs::GLOBAL_POSITION, &gps);
            
            // parse out the velocity and trigger that callback
            LocalFrameMessage vel(timestamp, float(gpi_msg.vx) / 100, float(gpi_msg.vy) / 100, float(gpi_msg.vz) / 100);
            notify_message_listeners(MessageIDs::LOCAL_VELOCITY, &vel);
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t hrt_msg;
            memset(&hrt_msg, 0, sizeof(mavlink_heartbeat_t));
            mavlink_msg_heartbeat_decode(&msg, &hrt_msg);

            uint32_t timestamp = 0.0;
            _armed = ((hrt_msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false);
            _hitl_enabled = ((hrt_msg.base_mode & MAV_MODE_FLAG_HIL_ENABLED) ? true : false);

            // determine if want to broadcast all current mode types
            // not just boolean on manual
            bool guided_mode = false;

            // extract whether or not we are in offboard mode for PX4
            // (the main mode)
            uint32_t main_mode = (hrt_msg.custom_mode & 0x000F0000) >> 16;
            if (main_mode == (uint32_t)MainMode::PX4_MODE_OFFBOARD)
            {
                guided_mode = true;
            }
            StateMessage state(timestamp, _armed, guided_mode, hrt_msg.system_status);
            notify_message_listeners(MessageIDs::STATE, &state);
            _target_system = msg.sysid;
            _target_component = msg.compid;
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_local_position_ned_t lpn_msg;
            memset(&lpn_msg, 0, sizeof(mavlink_local_position_ned_t));
            mavlink_msg_local_position_ned_decode(&msg, &lpn_msg);

            uint32_t timestamp = lpn_msg.time_boot_ms / 1000.0;
            // parse out the local positin and trigger that callback
            LocalFrameMessage pos(timestamp, lpn_msg.x, lpn_msg.y, lpn_msg.z);
            notify_message_listeners(MessageIDs::LOCAL_POSITION, &pos);

            // parse out the velocity and trigger that callback
            LocalFrameMessage vel(timestamp, lpn_msg.vx, lpn_msg.vy, lpn_msg.vz);
            notify_message_listeners(MessageIDs::LOCAL_VELOCITY, &vel);
            break;
        }
        case MAVLINK_MSG_ID_HOME_POSITION:
        {
            mavlink_home_position_t hp_msg;
            mavlink_msg_home_position_decode(&msg, &hp_msg);

            uint32_t timestamp = 0.0;
            GlobalFrameMessage home(timestamp, float(hp_msg.latitude) / 1e7, float(hp_msg.longitude) / 1e7, float(hp_msg.altitude) / 1000);
            notify_message_listeners(MessageIDs::GLOBAL_HOME, &home);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_IMU:
        {
            mavlink_scaled_imu_t si_msg;
            memset(&si_msg, 0, sizeof(mavlink_scaled_imu_t));
            mavlink_msg_scaled_imu_decode(&msg, &si_msg);

            uint32_t timestamp = si_msg.time_boot_ms / 1000.0;
            // break out the message into its respective messages for here
            BodyFrameMessage accel(timestamp, si_msg.xacc / 1000.0, si_msg.yacc / 1000.0, si_msg.zacc / 1000.0);  // units -> [mg]
            notify_message_listeners(MessageIDs::RAW_ACCELEROMETER, &accel);

            BodyFrameMessage gyro(timestamp, si_msg.xgyro / 1000.0, si_msg.ygyro / 1000.0, si_msg.zgyro / 1000.0);  // units -> [millirad/sec]
            notify_message_listeners(MessageIDs::RAW_GYROSCOPE, &gyro);
            break;
        }
        case MAVLINK_MSG_ID_RAW_IMU:
        {
            mavlink_raw_imu_t rimu_msg;
            memset(&rimu_msg, 0, sizeof(mavlink_raw_imu_t));
            mavlink_msg_raw_imu_decode(&msg, &rimu_msg);

            uint32_t timestamp = rimu_msg.time_usec / 1000.0;
            RAWIMUSensorMessage rawMsg(timestamp, rimu_msg.xacc, rimu_msg.yacc, rimu_msg.zacc,
                rimu_msg.xgyro, rimu_msg.ygyro, rimu_msg.zgyro,
                rimu_msg.xmag, rimu_msg.ymag, rimu_msg.zmag);
            notify_message_listeners(MessageIDs::RAW_IMU_SENSOR, &rawMsg);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
        {
            mavlink_scaled_pressure_t sp_msg;
            memset(&sp_msg, 0, sizeof(mavlink_scaled_pressure_t));
            mavlink_msg_scaled_pressure_decode(&msg, &sp_msg);

            uint32_t timestamp = sp_msg.time_boot_ms / 1000.0;
            BodyFrameMessage pressure(timestamp, 0, 0, sp_msg.press_abs);  // unit is [hectopascal]
            notify_message_listeners(MessageIDs::BAROMETER, &pressure);
            break;
        }
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        {
            mavlink_distance_sensor_t ds_msg;
            memset(&ds_msg, 0, sizeof(mavlink_distance_sensor_t));
            mavlink_msg_distance_sensor_decode(&msg, &ds_msg);

            uint32_t timestamp = ds_msg.time_boot_ms / 1000.0;
            float direction = 0;
            // TODO: parse orientation
            // orientation = msg.orientation
            DistanceSensorMessage meas(timestamp,
                float(ds_msg.min_distance) / 100,
                float(ds_msg.max_distance) / 100, direction,
                float(ds_msg.current_distance) / 100,
                float(ds_msg.covariance) / 100);
            notify_message_listeners(MessageIDs::DISTANCE_SENSOR, &meas);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        {
            _receivingAttitudeQuaternion = true;

            mavlink_attitude_quaternion_t aq_msg;
            memset(&aq_msg, 0, sizeof(mavlink_attitude_quaternion_t));
            mavlink_msg_attitude_quaternion_decode(&msg, &aq_msg);

            uint32_t timestamp = aq_msg.time_boot_ms / 1000.0;
            // TODO: check if mask notifies us to ignore a field

            FrameMessage fm(timestamp, aq_msg.q1, aq_msg.q2, aq_msg.q3, aq_msg.q4);
            notify_message_listeners(MessageIDs::ATTITUDE, &fm);

            BodyFrameMessage gyro(timestamp, aq_msg.rollspeed, aq_msg.pitchspeed, aq_msg.yawspeed);
            notify_message_listeners(MessageIDs::RAW_GYROSCOPE, &gyro);
            break;
        }
		case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_gps_raw_int_t gri_msg;
			memset(&gri_msg, 0, sizeof(mavlink_gps_raw_int_t));
            mavlink_msg_gps_raw_int_decode(&msg, &gri_msg);
            uint32_t timestamp = gri_msg.time_usec / 1000.0;
            // parse out the gps position and trigger that callback
            GlobalFrameMessage gps(timestamp, gri_msg.lat  / (float)1E7, gri_msg.lon / (float)1E7, gri_msg.alt  / 1000.0);
            notify_message_listeners(MessageIDs::GLOBAL_POSITION, &gps);
            _gpsRawIntMessageAvailable = true;
            break;
        }
        case MAVLINK_MSG_ID_GPS_INPUT:
        {
            mavlink_gps_input_t gpsInput;
            memset(&gpsInput, 0, sizeof(mavlink_gps_input_t));
            mavlink_msg_gps_input_decode(&msg, &gpsInput);

            uint32_t timestamp = gpsInput.time_usec / 1000.0;
            GPSSensorMessage gpsMsg(timestamp, float(gpsInput.lat) / 1e7, float(gpsInput.lon) / 1e7, float(gpsInput.alt) / 1000,
                float(gpsInput.vn / 100), float(gpsInput.ve / 100), float(gpsInput.vd / 100));
            notify_message_listeners(MessageIDs::GPS_INPUT_SENSOR, &gpsMsg);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK:
            handleCommandAck(msg);
            break;

            // DEBUG
        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            mavlink_statustext_t st_msg;
            memset(&st_msg, 0, sizeof(mavlink_statustext_t));
            mavlink_msg_statustext_decode(&msg, &st_msg);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
        {
            handleAttitudeTarget(msg);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE:
        {
            if (_receivingAttitudeQuaternion) {
                return;
            }

            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);

            //cout << "Attitude(" << attitude.roll << "," << attitude.pitch << ", " << attitude.yaw << ")" << endl;
            break;
        }
        //Metrics typically displayed on a HUD for fixed wing aircraft.
        case MAVLINK_MSG_ID_VFR_HUD:
        {
            mavlink_vfr_hud_t vfrHud;
            mavlink_msg_vfr_hud_decode(&msg, &vfrHud);
            /*cout << "airspeed: " << vfrHud.airspeed << endl;
            cout << "groundSpeedFact: " << vfrHud.groundspeed << endl;
            cout << "climbRateFact: " << vfrHud.climb << endl;
            cout << "throttlePctFact: " << vfrHud.throttle << endl;*/
            break;
        }
        case MAVLINK_MSG_ID_ALTITUDE:
        {
            mavlink_altitude_t altitude;
            mavlink_msg_altitude_decode(&msg, &altitude);

            // If data from GPS is available it takes precedence over ALTITUDE message
            if (!_globalPositionIntMessageAvailable) {
                //cout << "altitude relative: " << altitude.altitude_relative << endl;
                if (!_gpsRawIntMessageAvailable) {
                    cout << "altitude_amsl: " << altitude.altitude_amsl << endl;
                }
            }
            break;
        }
        default:
        {
            //printf("Warning, did not handle message id %i\n", msg.msgid);
            break;
        }
    }
}

void MavlinkConnection::handleAttitudeTarget(mavlink_message_t& message)
{
    mavlink_attitude_target_t attitudeTarget;

    mavlink_msg_attitude_target_decode(&message, &attitudeTarget);

    float roll, pitch, yaw;
    mavlink_quaternion_to_euler(attitudeTarget.q, &roll, &pitch, &yaw);
    //printf("AttitudeTarget:%f, %f, %f\n", roll, pitch, yaw);
}

void MavlinkConnection::handleCommandAck(mavlink_message_t& message)
{
    bool showError = true;
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&message, &ack);

    ArcResultMessage ackMsg(ack.command, ack.result);
    notify_message_listeners(MessageIDs::COMMANDACKRESULT, &ackMsg);

    if (ack.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES && ack.result != MAV_RESULT_ACCEPTED) {
        cout<< "Vehicle responded to MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES with error(" << ack.result << "). Setting no capabilities." << endl;
    }

    if (ack.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            // The vehicle should be sending a PROTOCOL_VERSION message in a mavlink 2 packet. This may or may not make it through the pipe.
            // So we wait for it to come and timeout if it doesn't.
//            if (!_mavlinkProtocolRequestComplete) {
//                QTimer::singleShot(1000, this, &Vehicle::_protocolVersionTimeOut);
//            }
        }
        else {
            cout << "Vehicle responded to MAV_CMD_REQUEST_PROTOCOL_VERSION with error(" << ack.result <<  ")." << endl;
        }
    }

    if (ack.command == MAV_CMD_DO_SET_ROI_LOCATION) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            cout << "MAV_CMD_DO_SET_ROI_LOCATION" << endl;
        }
    }

    if (ack.command == MAV_CMD_DO_SET_ROI_NONE) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            cout << "MAV_CMD_DO_SET_ROI_NONE" << endl;
        }
    }

#if !defined(NO_ARDUPILOT_DIALECT)
    if (ack.command == MAV_CMD_FLASH_BOOTLOADER && ack.result == MAV_RESULT_ACCEPTED) {
        cout << "Bootloader flash succeeded" << endl;
    }
#endif

    if (showError) {
        switch (ack.result) {
        case MAV_RESULT_TEMPORARILY_REJECTED:
            cout << ack.command << " command temporarily rejected" << endl;
            break;
        case MAV_RESULT_DENIED:
            cout << ack.command << " command denied" << endl;
            break;
        case MAV_RESULT_UNSUPPORTED:
            cout << ack.command << " command not supported" << endl;
            break;
        case MAV_RESULT_FAILED:
            cout << ack.command << " command failed" << endl;
            break;
        case MAV_RESULT_ACCEPTED:
            break;
        default:
            // Do nothing
            //cout << ack.command << ":" << ack.result << endl;
            break;
        }
    }
}

void MavlinkConnection::command_loop()
{
    /*
    Main loop for sending commands.

    Loop that is run a separate thread to be able to send messages to the
    target drone.  Uses the message queue `_out_msg_queue` as the
    queue of messages to run.
    */

    /*
     default to sending a velocity command to (0,0,0)
     this needs to be sending commands at a rate of at lease 2Hz in order
     for PX4 to allow a switch into offboard control.
     */

    if (writing_status)
    {
        cout << "write thread already running" << endl;
        return;
    }

    mavlink_set_position_target_local_ned_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.type_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AX | IGNORE_AY | IGNORE_AZ | IGNORE_YAW_RATE;
    packet.target_system = _target_system;
    packet.target_component = autopilot_id;
    packet.coordinate_frame = MAV_FRAME_LOCAL_NED;
    packet.x = 0.0;
    packet.y = 0.0;
    packet.z = 0.0;
    packet.vx       = 0.0;
    packet.vy       = 0.0;
    packet.vz       = 0.0;
    packet.afx = 0.0;
    packet.afy = 0.0;
    packet.afz = 0.0;
    packet.yaw = 0.0;
    packet.yaw_rate = 0.0;
    
    mavlink_message_t high_rate_command;
    memset(&high_rate_command, 0, sizeof(mavlink_message_t));
    mavlink_msg_set_position_target_local_ned_encode(_target_system, _target_component, &high_rate_command, &packet);
    send_message_immediately(high_rate_command);
    writing_status = true;
    
    dl_time_t last_ping_time{};
    time_t last_write_time = time(nullptr);
    while(_running)
    {
        // empty out the queue of pending messages
        // NOTE: Queue class is synchronized and is thread safe already!
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(Message));
        
        msg_queue_mutex.lock();
        while (_out_msg_queue.size() > 0)
        {
            memcpy(&msg, &_out_msg_queue.front(), sizeof(mavlink_message_t));
            _out_msg_queue.pop();
            // either set this is as the high rate command
            // to repeatedly send or send it immediately
            if (msg.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED || msg.msgid == MAVLINK_MSG_ID_SET_ATTITUDE_TARGET)
            {
                memcpy(&high_rate_command, &msg, sizeof(mavlink_message_t));
            }
            // either way, want to send this command immediately
            send_message_immediately(msg);
        }
        msg_queue_mutex.unlock();
        
        // rate limit the loop
        // though only do this after we have handled any new messages to send
        // this ensures messages get sent off immediately
        time_t current_time = time(nullptr);
        if ((current_time - last_write_time) < 1.0 / _send_rate)
        {
            continue;
        }
        last_write_time = current_time;

        //if (elapsed_since_s(last_ping_time) >= _ping_interval_s) {
        //    uint64_t now = static_cast<uint64_t>(elapsed_s() * 1e6);
        //    mavlink_message_t message;
        //    memset(&message, 0, sizeof(mavlink_message_t));
        //    mavlink_msg_ping_pack(
        //        _target_system,
        //        _target_component,
        //        &message,
        //        now,
        //        0,
        //        0,
        //        0); // to all
        //    send_message_immediately(message);
        //    last_ping_time = steady_time();
        //}

        // continually want to send the high rate command
        send_message_immediately(high_rate_command);
    }
    writing_status = false;
}

bool MavlinkConnection::wait_for_message(mavlink_message_t &msg)
{
    /*
    Wait for a new mavlink message calls mavlink's blocking read function to read
    a next message, blocking for up to a timeout of 1s.

    Returns:
        Mavlink message that was read or `None` if the message was invalid.
    */

    // NOTE: this returns a mavlink message
    // this function should not be called outside of this class!
    if (_master->recv_match(&msg, true, 1))
    {
        // send a heartbeat message back, since this needs to be
        // constantly sent so the autopilot knows this exists
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
        {
            // send -> type, autopilot, base mode, custom mode, system status
            /*mavlink_heartbeat_t packet;
            memset(&packet, 0, sizeof(packet));
            packet.custom_mode = 0;
            packet.type = MAV_TYPE_GCS;
            packet.autopilot = MAV_AUTOPILOT_GENERIC;
            packet.base_mode = 0;
            packet.system_status = 0;
            packet.mavlink_version = 3;*/
            
            mavlink_message_t outmsg;
            //mavlink_msg_heartbeat_encode(_target_system, _target_component, &outmsg, &packet);

            mavlink_msg_heartbeat_pack(
                _target_system,
                _target_component,
                &outmsg,
                MAV_TYPE_ONBOARD_CONTROLLER,
                MAV_AUTOPILOT_GENERIC,
                0,
                0,
                0);
            send_message_immediately(outmsg);
        }

        return true;
    }
    return false;
}

void MavlinkConnection::start()
{
    // start the main thread
    _running = true;

    /* start the command loop
    this is only needed when working with PX4 and we need to enforce
    a certain rate of messages being sent
   */
    if (_using_px4)
    {
        _write_handle = new thread(&MavlinkConnection::command_loop, this);

        // wait for it to be started
        while (!writing_status)
            usleep(2); // 10Hz

        _write_handle_daemon = true;
    }

    // start the dispatch loop, either threaded or not
    if (_threaded)
    {
        _read_handle = new thread(&MavlinkConnection::dispatch_loop, this);
        cout << "CHECK FOR MESSAGES" << endl;
    
        while ( !_target_system )
        {
            if ( !_running )
                return;
            usleep(500000); // check at 2Hz
        }
    
        cout << "Found" << endl;
        
        _read_handle_daemon = true;
    }
    else
    {
        _read_handle = nullptr;
        dispatch_loop();
    }

    // Wait for system to connect via heartbeat.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    if (_write_handle_daemon)
    {
        _write_handle->join();
    }
    
    if (_read_handle_daemon)
    {
        _read_handle->join();
    }
}

void MavlinkConnection::stop()
{
    // stop the dispatch and command while loops
    _running = false;
    writing_status = false;
    sleep(2);

    // NOTE: no need to call join on the threads
    // as both threads are daemon threads

    printf("Closing connection ...\r\n");
    if (_master) {
        delete _master;
        _master = nullptr;
    }
}

void MavlinkConnection::send_message(mavlink_message_t msg)
{
    /*
    Send a given mavlink message to the drone. If connected with a PX4 autopilot,
    add the MAVLinkMessage to the command queue to be handled by the command loop
    (running in the write thread).  Otherwise immediately send the message.

    :param msg: MAVLinkMessage to be sent to the drone
    */

    /* if we are using PX4, means we are also using out command loop
     in a separate thread and therefore need to send the data to that
     thread using the queue
    
     if we are not using PX4, then just immediately send the message
     */
    if (_using_px4)
    {
        msg_queue_mutex.lock();
        _out_msg_queue.push(msg);
        msg_queue_mutex.unlock();
    }
    else
    {
        send_message_immediately(msg);
    }
}

void MavlinkConnection::send_message_immediately(mavlink_message_t msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {0};
    memset(buffer, 0, MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    _master->write(buffer, len);
}

void MavlinkConnection::send(const vector<uint8_t>& packet)
{
  _master->write(&packet[0], (int)packet.size());
}

void MavlinkConnection::send_long_command(uint16_t command_type, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    /*
    Packs and sends a Mavlink COMMAND_LONG message

    Args:
        command_type: the command type, as defined by MAV_CMD_*
        param1: param1 as defined by the specific command
        param2: param2 as defined by the specific command (default: {0})
        param3: param3 as defined by the specific command (default: {0})
        param4: param4 as defined by the specific command (default: {0})
        param5: param5 (x) as defined by the specific command (default: {0})
        param6: param6 (y) as defined by the specific command (default: {0})
        param7: param7 (z) as defined by the specific command (default: {0})
    */
    mavlink_command_long_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command_type;
    packet.target_system = _target_system;
    packet.target_component = _target_component;
    packet.confirmation = true; //may want this as an input.... used for repeat messages
    
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_command_long_encode(_own_system, _own_component, &msg, &packet);
    send_message(msg);
}

void MavlinkConnection::arm()
{
    send_long_command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196);
}

void MavlinkConnection::disarm()
{
    send_long_command(MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196);
}

void MavlinkConnection::take_control()
{
    send_long_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)MainMode::PX4_MODE_OFFBOARD, 0);
}

void MavlinkConnection::release_control()
{
    send_long_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)MainMode::PX4_MODE_MANUAL, 0);
}

void MavlinkConnection::cmd_attitude_target_send(uint16_t type_mask, float roll, float pitch, float yaw, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust)
{  
    // convert the attitude to a quaternion
    uint32_t time_boot_ms = elapsed_s() * 1e3;
    FrameMessage frame_msg(time_boot_ms, roll, pitch, yaw);
    mavlink_message_t msg;
    mavlink_msg_set_attitude_target_pack(
        _own_system,
        _own_component,
        &msg,
        time_boot_ms,
        _target_system,
        _target_component,
        type_mask,
        frame_msg.q(),
        body_roll_rate,
        body_pitch_rate,
        body_yaw_rate,
        thrust);

    send_message(msg);
}

void MavlinkConnection::cmd_attitude(float roll, float pitch, float yaw, float thrust)
{
    uint16_t type_mask = IGNORE_BODY_ROLL_RATE | IGNORE_BODY_PITCH_RATE | IGNORE_BODY_YAW_RATE;
    cmd_attitude_target_send(type_mask, roll, pitch, yaw, 0, 0, 0, thrust);
}

void MavlinkConnection::cmd_attitude_rate(float roll_rate, float pitch_rate, float yaw_rate, float thrust)
{
    uint16_t type_mask = IGNORE_ATTITUDE;
    cmd_attitude_target_send(type_mask, 0.0, 0.0, 0.0, roll_rate, pitch_rate, yaw_rate, thrust);
}

void MavlinkConnection::cmd_moment(float roll_moment, float pitch_moment, float yaw_moment, float thrust, time_t t)
{
    uint16_t type_mask = IGNORE_ATTITUDE;
    cmd_attitude_target_send(type_mask, 0.0, 0.0, 0.0, roll_moment, pitch_moment, yaw_moment, thrust);
}

void MavlinkConnection::msg_set_position_target_local_ned_pack(uint16_t mask, float n, float e, float d, float vn, float ve, float vd, float an, float ae, float ad, float heading)
{
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        _own_system,
        _own_component,
        &msg,
        elapsed_s() * 1e3,
        _target_system,
        _target_component,
        MAV_FRAME_LOCAL_NED,
        mask, 0, 0, 0, vn, ve, vd, 0, 0, 0, heading * M_DEG_TO_RAD, 0
    );

    send_message(msg);
}

void MavlinkConnection::cmd_velocity(float vn, float ve, float vd, float heading)
{
    uint16_t mask = IGNORE_X | IGNORE_Y | IGNORE_Z | IGNORE_AX | IGNORE_AY | IGNORE_AZ | IGNORE_YAW_RATE;
    msg_set_position_target_local_ned_pack(mask, 0, 0, 0, vn, ve, vd, 0, 0, 0, heading);
}

void MavlinkConnection::cmd_acceleration(float ax, float ay, float az, float heading)
{
    uint16_t mask = IGNORE_X | IGNORE_Y | IGNORE_Z | IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_YAW_RATE;
    msg_set_position_target_local_ned_pack(mask, 0, 0, 0, 0, 0, 0, ax, ay , az, heading);
}

void MavlinkConnection::cmd_position(float n, float e, float d, float heading)
{
    uint16_t mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AX | IGNORE_AY | IGNORE_AZ | IGNORE_YAW_RATE;
    msg_set_position_target_local_ned_pack(mask, n, e, d, 0, 0, 0, 0, 0, 0, heading);
}

void MavlinkConnection::cmd_position(V4F p)
{
    cmd_position(p[0], p[1], p[2], p[3]);
}

void MavlinkConnection::cmd_controls(float *controls, time_t t)
{
    mavlink_set_actuator_control_target_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.time_usec = (t * 1000000);
    packet.group_mlx = 1;
    packet.target_system = _target_system;
    packet.target_component = autopilot_id;
    
    // ensure controls_out is of length 8 even if controls isn't
    memset(packet.controls, 0, sizeof(float) * 8);
    mav_array_memcpy(packet.controls, controls, sizeof(float)*8);
    
    mavlink_message_t msg;
    mavlink_msg_set_actuator_control_target_encode(_own_system, _own_component, &msg, &packet);
    
    send_message(msg);
}

void MavlinkConnection::takeoff(float n, float e, float d)
{
    /*
     for mavlink to PX4 need to specify the NED location for landing
     since connection doesn't keep track of this info, have drone send it
     abstract away that part in the drone class
     */
    send_long_command(MAV_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
}

void MavlinkConnection::land(float n, float e)
{
    // for mavlink to PX4 need to specify the NED location for landing
    // since connection doesn't keep track of this info, have drone send it
    // abstract away that part in the drone class
    send_long_command(MAV_CMD_NAV_LAND, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
}

void MavlinkConnection::set_home_position(float lat, float lon, float alt)
{
    send_long_command(MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, lat, lon, alt);
}

void MavlinkConnection::local_position_target(float n, float e, float d, time_t t)
{
    cmd_position(n, e, d, 0.0);
}

void MavlinkConnection::local_velocity_target(float vn, float ve, float vd, time_t t)
{
    cmd_velocity(vn, ve, vd, 0);
}

void MavlinkConnection::local_acceleration_target(float an, float ae, float ad, time_t t)
{
    cmd_acceleration(an, ae, ad, 0);
}

void MavlinkConnection::attitude_target(float roll, float pitch, float yaw, time_t t)
{
    uint16_t type_mask = IGNORE_BODY_ROLL_RATE | IGNORE_BODY_PITCH_RATE | IGNORE_BODY_YAW_RATE| IGNORE_THROTTLE;
    cmd_attitude_target_send(type_mask, roll, pitch, yaw, 0, 0, 0, 0);
}

void MavlinkConnection::body_rate_target(float p, float q, float r, time_t t)
{
    uint16_t type_mask = IGNORE_ATTITUDE | IGNORE_THROTTLE;
    cmd_attitude_target_send(type_mask, 0, 0, 0, p, q, r, 0);
}

void MavlinkConnection::set_sub_mode(int sub_mode)
{
    send_long_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)MainMode::PX4_MODE_OFFBOARD, sub_mode);
}

void MavlinkConnection::set_notify_callback(notify_message_callback fn)
{
    _notify_message_callback = fn;
}

void MavlinkConnection::notify_message_listeners(MessageIDs name, void *msg)
{
    (_drone->*_notify_message_callback)(name, msg);
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
void MavlinkConnection::cmd_offboard_control(bool flag)
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = { 0 };
    com.target_system    = _target_system;
    com.target_component = autopilot_id;
    com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation     = true;
    com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop
    
    // Encode
    mavlink_message_t message;
//    mavlink_msg_command_long_encode_chan(_target_system, _target_component, _target_channel, &message, &com);
    mavlink_msg_command_long_encode(_own_system, _own_component, &message, &com);

    // Send the message
    send_message_immediately(message);
}

void MavlinkConnection::make_command_flight_mode(FlightMode flight_mode)
{
    /*const uint8_t flag_safety_armed = is_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
    const uint8_t flag_hitl_enabled = _hitl_enabled ? MAV_MODE_FLAG_HIL_ENABLED : 0;

    const uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | flag_safety_armed | flag_hitl_enabled;*/
    const uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    // Note: the safety flag is not needed in future versions of the PX4 Firmware
    //       but want to be rather safe than sorry.
    PX4_CUSTOM_MAIN_MODE custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_AUTO;
    PX4_CUSTOM_SUB_MODE_AUTO custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO(0);

    switch (flight_mode) {
    case FlightMode::Hold:
        custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
        break;
    case FlightMode::ReturnToLaunch:
        custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_RTL;
        break;
    case FlightMode::Takeoff:
        custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
        break;
    case FlightMode::Land:
        custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_LAND;
        break;
    case FlightMode::Mission:
        custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
        break;
    case FlightMode::FollowMe:
        custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
        break;
    case FlightMode::Offboard:
        custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_OFFBOARD;
        break;
    case FlightMode::Manual:
        custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_MANUAL;
        break;
    case FlightMode::Posctl:
        custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_POSCTL;
        break;
    case FlightMode::Altctl:
        custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_ALTCTL;
        break;
    case FlightMode::Rattitude:
        custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_RATTITUDE;
        break;
    case FlightMode::Acro:
        custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_ACRO;
        break;
    case FlightMode::Stabilized:
        custom_mode = PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_STABILIZED;
        break;
    default:
        cout << "Unknown Flight mode." << endl;
        return;
    }

    send_long_command(MAV_CMD_DO_SET_MODE, float(mode), float(custom_mode), float(custom_sub_mode));
}