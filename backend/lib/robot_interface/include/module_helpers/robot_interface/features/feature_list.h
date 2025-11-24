#pragma once

#include <cstdint>

namespace aergo::module::helpers::robot_interface
{
    /// @brief Features that are supported by the robot interface.
    /// Each feature represents a specific capability of the robot that can be controlled via the robot interface.
    enum class RobotFeature : uint64_t
    {
        NONE = 0,                     // No feature / invalid feature
        ROBOT_CONTROL = 1,            // General robot control feature, always required to be supported
        SIMPLER_GRIPPER_CONTROL = 2,  // Control of the robot's gripper with simple open/close commands
        WELDING = 3                   // Control of the welding tool
    };
}