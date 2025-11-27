#pragma once

#include <cstdint>
#include <vector>

namespace aergo::module::helpers::robot_interface::robot_control
{
    struct Vector3
    {
        double x;
        double y;
        double z;
    };

    struct Quaternion
    {
        double x;
        double y;
        double z;
        double w;
    };

    struct Pose
    {
        Vector3 position;       // Position in world coordinates, in meters.
        Quaternion orientation; // Orientation as a normalized quaternion representing the rotation from local coordinates to world coordinates (v_world = orientation * v_local)
    };

    enum class OrientationType : uint8_t
    { 
        FIXED,    // Keep the orientation constant throughout the movement (start orientation is used)
        TANGENT   // Align the orientation with the tangent of the path through the movement
    };

    enum class RobotStatus : uint8_t
    {
        IDLE,
        MOVING,
        ERROR
    };

    struct JointRange
    {
        double min; // in radians
        double max; // in radians
    };

    struct RobotSpecs
    {
        double max_velocity_linear;           // in m/s
        double max_velocity_angular;          // in rad/s
        double max_acceleration_linear;       // in m/s²
        double max_acceleration_angular;      // in rad/s²

        uint8_t num_joints;
        std::vector<JointRange> joint_limits; // joint limits for each joint in radians
    };

}