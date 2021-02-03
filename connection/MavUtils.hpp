#pragma once

constexpr auto IGNORE_X = (1 << 0);
constexpr auto IGNORE_Y = (1 << 1);
constexpr auto IGNORE_Z = (1 << 2);
constexpr auto IGNORE_VX = (1 << 3);
constexpr auto IGNORE_VY = (1 << 4);
constexpr auto IGNORE_VZ = (1 << 5);
constexpr auto IGNORE_AX = (1 << 6);
constexpr auto IGNORE_AY = (1 << 7);
constexpr auto IGNORE_AZ = (1 << 8);
constexpr auto IGNORE_YAW_RATE = (1 << 11);
constexpr auto IGNORE_BODY_ROLL_RATE = (1 << 0);
constexpr auto IGNORE_BODY_PITCH_RATE = (1 << 1);
constexpr auto IGNORE_BODY_YAW_RATE = (1 << 2);
constexpr auto IGNORE_ATTITUDE = (1 << 7);
constexpr auto IGNORE_THROTTLE = (1 << 6);

enum class ConnectionType
{
    /*
    Different possible connection types.
    NOTE: Right now the only implemented type is PX4 mavlink
    */
    MAVLINK_PX4 = 1,
    // MAVLINK_APM = 2,
    // PARROT = 3,
    // DJI = 4
};

enum class MainMode
{
    //Constant which isn't defined in Mavlink but is useful for PX4
    PX4_MODE_MANUAL = 1,
    PX4_MODE_OFFBOARD = 6
};

enum class PlaneMode
{
    /*
     Constant which isn't defined in Mavlink but useful when dealing with
    the airplane simulation
     */
    SUB_MODE_LONGITUDE = 1,
    SUB_MODE_LATERAL = 2,
    SUB_MODE_STABILIZED = 3
};

enum class PositionMask
{
    //Useful masks for sending commands used in set_position_target_local_ned
    MASK_IGNORE_POSITION = 0x007,
    MASK_IGNORE_VELOCITY = 0x038,
    MASK_IGNORE_ACCELERATION = 0x1C0,
    MASK_IGNORE_YAW = 0x400,
    MASK_IGNORE_YAW_RATE = 0x800,
    MASK_IS_FORCE = (1 << 9),
    MASK_IS_TAKEOFF = 0x1000,
    MASK_IS_LAND = 0x2000,
    MASK_IS_LOITER = 0x3000
};

inline PositionMask operator|(PositionMask p, PositionMask q)
{
    uint16_t mask = (uint16_t)p, mask1 = (uint16_t)q;
    return (PositionMask)(mask | mask1);
}
enum class AttitudeMask
{
    MASK_IGNORE_ATTITUDE = 0b10000000,
    MASK_IGNORE_RATES = 0b00000111
};

enum class FlightMode {
    Unknown,
    Ready,
    Takeoff,
    Hold,
    Mission,
    ReturnToLaunch,
    Land,
    Offboard,
    FollowMe,
    Manual,
    Altctl,
    Posctl,
    Acro,
    Rattitude,
    Stabilized,
};

enum class PX4_CUSTOM_MAIN_MODE {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL,
    PX4_CUSTOM_MAIN_MODE_POSCTL,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_ACRO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_MAIN_MODE_STABILIZED,
    PX4_CUSTOM_MAIN_MODE_RATTITUDE
};

enum class PX4_CUSTOM_SUB_MODE_AUTO {
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET
};
