#include "module_helpers/robot_interface/features/robot_control/messages.h"

#include "module_helpers/serialization_helper/serialization_helper.h"

#include <stdexcept>
#include <cstring>

using namespace aergo::module::helpers::robot_interface::robot_control;
namespace ser = aergo::module::helpers::serialization_helper::serialization;
namespace des = aergo::module::helpers::serialization_helper::deserialization;

static_assert(sizeof(double) == 8, "Expected double to be 8 bytes for serialization");



/// @brief Pushes a Pose object onto the serialization buffer.
/// Layout (payload):
///    [f64 position.x][f64 position.y][f64 position.z]
///    [f64 orientation.w][f64 orientation.x][f64 orientation.y][f64 orientation.z]
/// @param buffer The buffer to serialize the pose into.
/// @param pose The pose object to serialize.
void pushPose(std::vector<std::byte>& buffer, const Pose& pose)
{
    ser::push<double>(buffer, pose.position.x);
    ser::push<double>(buffer, pose.position.y);
    ser::push<double>(buffer, pose.position.z);
    ser::push<double>(buffer, pose.orientation.w);
    ser::push<double>(buffer, pose.orientation.x);
    ser::push<double>(buffer, pose.orientation.y);
    ser::push<double>(buffer, pose.orientation.z);
}



/// ============================
/// Start Request Serializations
/// ============================

std::vector<std::byte>& start::requests::serialization::moveJoint(std::vector<std::byte>& buffer, Span<const double> joint_targets, double speed, double acceleration)
{
    buffer.clear();

    if (joint_targets.size() > MAX_SUPPORTED_JOINTS)
    {
        throw std::invalid_argument("moveJoint: joint_targets size exceeds maximum supported joint count of 16");
    }

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(RequestType::MoveJoint));
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(joint_targets.size()));
    for (double target : joint_targets)
    {
        ser::push<double>(buffer, target);
    }
    ser::push<double>(buffer, speed);
    ser::push<double>(buffer, acceleration);
    return buffer;
}


std::vector<std::byte>& start::requests::serialization::moveLinear(std::vector<std::byte>& buffer, Pose pose_target, double speed, double acceleration)
{
    buffer.clear();

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(RequestType::MoveLinear));
    pushPose(buffer, pose_target);
    ser::push<double>(buffer, speed);
    ser::push<double>(buffer, acceleration);
    return buffer;
}


std::vector<std::byte>& start::requests::serialization::moveArc(std::vector<std::byte>& buffer, Pose pose_through, Pose pose_target, double speed, double acceleration, OrientationType orientation_type, bool as_circle, double circle_percentage)
{
    buffer.clear();

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(RequestType::MoveArc));
    pushPose(buffer, pose_through);
    pushPose(buffer, pose_target);
    ser::push<double>(buffer, speed);
    ser::push<double>(buffer, acceleration);
    ser::push<uint8_t>(buffer, static_cast<uint8_t>(orientation_type));
    ser::push<bool>(buffer, as_circle);
    ser::push<double>(buffer, circle_percentage);
    return buffer;
}


std::vector<std::byte>& start::requests::serialization::moveTrajectory(std::vector<std::byte>& buffer, Span<const Pose> pose_targets, double speed, double acceleration, OrientationType orientation_type)
{
    buffer.clear();

    if (pose_targets.size() > MAX_SUPPORTED_POSES)
    {
        throw std::invalid_argument("moveTrajectory: pose_targets size exceeds maximum supported count of 32768");
    }

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(RequestType::MoveTrajectory));
    ser::push<uint16_t>(buffer, static_cast<uint16_t>(pose_targets.size()));
    for (const Pose& pose : pose_targets)
    {
        pushPose(buffer, pose);
    }
    ser::push<double>(buffer, speed);
    ser::push<double>(buffer, acceleration);
    ser::push<uint8_t>(buffer, static_cast<uint8_t>(orientation_type));
    return buffer;
}


std::vector<std::byte>& start::requests::serialization::getRobotSpecs(std::vector<std::byte>& buffer)
{
    buffer.clear();

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(RequestType::GetRobotSpecs));
    return buffer;
}




/// ==============================
/// Start Request Deserializations
/// ==============================

bool deserializePose(des::BufferReader& reader, Pose& out_pose)
{
    if (!reader.read<double>(out_pose.position.x) ||
        !reader.read<double>(out_pose.position.y) ||
        !reader.read<double>(out_pose.position.z) ||
        !reader.read<double>(out_pose.orientation.w) ||
        !reader.read<double>(out_pose.orientation.x) ||
        !reader.read<double>(out_pose.orientation.y) ||
        !reader.read<double>(out_pose.orientation.z))
    {
        return false;
    }
    return true;
}


bool deserializeMoveJointRequest(des::BufferReader& reader, start::requests::deserialization::MoveJointRequest& out_request)
{
    uint8_t joint_count{};
    if (!reader.read<uint8_t>(joint_count) || joint_count > start::requests::MAX_SUPPORTED_JOINTS)
    {
        return false;
    }

    out_request.joint_targets.resize(joint_count);
    for (uint8_t i = 0; i < joint_count; ++i)
    {
        if (!reader.read<double>(out_request.joint_targets[i]))
        {
            return false;
        }
    }

    if (!reader.read<double>(out_request.speed) ||
        !reader.read<double>(out_request.acceleration))
    {
        return false;
    }

    return true;
}


bool deserializeMoveLinearRequest(des::BufferReader& reader, start::requests::deserialization::MoveLinearRequest& out_request)
{
    if (!deserializePose(reader, out_request.pose_target))
    {
        return false;
    }

    if (!reader.read<double>(out_request.speed) ||
        !reader.read<double>(out_request.acceleration))
    {
        return false;
    }

    return true;
}


bool deserializeMoveArcRequest(des::BufferReader& reader, start::requests::deserialization::MoveArcRequest& out_request)
{
    if (!deserializePose(reader, out_request.pose_through) ||
        !deserializePose(reader, out_request.pose_target))
    {
        return false;
    }

    if (!reader.read<double>(out_request.speed) ||
        !reader.read<double>(out_request.acceleration))
    {
        return false;
    }

    uint8_t orientation_type_u8{};
    if (!reader.read<uint8_t>(orientation_type_u8))
    {
        return false;
    }
    out_request.orientation_type = static_cast<OrientationType>(orientation_type_u8);

    if (!reader.read<bool>(out_request.as_circle) ||
        !reader.read<double>(out_request.circle_percentage))
    {
        return false;
    }

    return true;
}


bool deserializeMoveTrajectoryRequest(des::BufferReader& reader, start::requests::deserialization::MoveTrajectoryRequest& out_request)
{
    uint16_t pose_count{};
    if (!reader.read<uint16_t>(pose_count) || pose_count > start::requests::MAX_SUPPORTED_POSES)
    {
        return false;
    }

    out_request.pose_targets.resize(pose_count);
    for (uint16_t i = 0; i < pose_count; ++i)
    {
        if (!deserializePose(reader, out_request.pose_targets[i]))
        {
            return false;
        }
    }

    if (!reader.read<double>(out_request.speed) ||
        !reader.read<double>(out_request.acceleration))
    {
        return false;
    }

    uint8_t orientation_type_u8{};
    if (!reader.read<uint8_t>(orientation_type_u8))
    {
        return false;
    }
    out_request.orientation_type = static_cast<OrientationType>(orientation_type_u8);

    return true;
}


bool deserializeRobotSpecsRequest(des::BufferReader& reader, start::requests::deserialization::GetRobotSpecsRequest& out_request)
{
    // no payload to read
    return true;
}


bool start::requests::deserialization::deserialize(BufferReader& reader, RequestVariant& out_request)
{
    // read version
    uint8_t version{};
    if (!reader.read<uint8_t>(version) || version != ROBOT_CONTROL_MAJOR_VERSION)
    {
        return false; // version mismatch
    }

    // read request type
    uint8_t request_type_u8{};
    if (!reader.read<uint8_t>(request_type_u8))
    {
        return false; // parse error
    }
    RequestType request_type = static_cast<RequestType>(request_type_u8);
    switch (request_type)
    {
        case RequestType::MoveJoint:
        {
            MoveJointRequest request;
            if (!deserializeMoveJointRequest(reader, request))
            {
                return false;
            }
            out_request = std::move(request);
            return true;
        }
        case RequestType::MoveLinear:
        {
            MoveLinearRequest request;
            if (!deserializeMoveLinearRequest(reader, request))
            {
                return false;
            }
            out_request = std::move(request);
            return true;
        }
        case RequestType::MoveArc:
        {
            MoveArcRequest request;
            if (!deserializeMoveArcRequest(reader, request))
            {
                return false;
            }
            out_request = std::move(request);
            return true;
        }
        case RequestType::MoveTrajectory:
        {
            MoveTrajectoryRequest request;
            if (!deserializeMoveTrajectoryRequest(reader, request))
            {
                return false;
            }
            out_request = std::move(request);
            return true;
        }
        case RequestType::GetRobotSpecs:
        {
            GetRobotSpecsRequest request;
            if (!deserializeRobotSpecsRequest(reader, request))
            {
                return false;
            }
            out_request = std::move(request);
            return true;
        }
        default:
            return false; // unknown type
    }
}





/// ==============================
/// Start Response Serializations
/// ==============================

std::vector<std::byte>& start::responses::serialization::robotSpecs(std::vector<std::byte>& buffer, const RobotSpecs& specs)
{
    buffer.clear();

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(ResponseType::RobotSpecs));
    ser::push<double>(buffer, specs.max_velocity_linear);
    ser::push<double>(buffer, specs.max_velocity_angular);
    ser::push<double>(buffer, specs.max_acceleration_linear);
    ser::push<double>(buffer, specs.max_acceleration_angular);
    ser::push<uint8_t>(buffer, specs.num_joints);
    for (const JointRange& joint_range : specs.joint_limits)
    {
        ser::push<double>(buffer, joint_range.min);
        ser::push<double>(buffer, joint_range.max);
    }
    return buffer;
}


/// ==============================
/// Start Response Deserializations
/// ==============================

bool start::responses::deserialization::deserializeRobotSpecs(BufferReader& reader, RobotSpecs& out_robot_specs)
{
    // read version
    uint8_t version{};
    if (!reader.read<uint8_t>(version) || version != ROBOT_CONTROL_MAJOR_VERSION)
    {
        return false; // version mismatch
    }

    // read request type
    uint8_t request_type_u8{};
    if (!reader.read<uint8_t>(request_type_u8))
    {
        return false; // parse error
    }
    ResponseType response_type = static_cast<ResponseType>(request_type_u8);
    if (response_type != ResponseType::RobotSpecs)
    {
        return false; // unknown type
    }

    if (!reader.read<double>(out_robot_specs.max_velocity_linear) ||
        !reader.read<double>(out_robot_specs.max_velocity_angular) ||
        !reader.read<double>(out_robot_specs.max_acceleration_linear) ||
        !reader.read<double>(out_robot_specs.max_acceleration_angular))
    {
        return false;
    }

    uint8_t num_joints{};
    if (!reader.read<uint8_t>(num_joints))
    {
        return false;
    }
    out_robot_specs.num_joints = num_joints;

    out_robot_specs.joint_limits.resize(num_joints);
    for (uint8_t i = 0; i < num_joints; ++i)
    {
        JointRange joint_range{};
        if (!reader.read<double>(joint_range.min) ||
            !reader.read<double>(joint_range.max))
        {
            return false;
        }
        out_robot_specs.joint_limits[i] = joint_range;
    }

    return true;
}




/// ============================
/// Update Request Serializations
/// ============================

std::vector<std::byte>& update::requests::serialization::moveRequest(std::vector<std::byte>& buffer, MoveRequest request_type)
{
    buffer.clear();

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint8_t>(buffer, static_cast<std::uint8_t>(request_type));
    return buffer;
}

/// ============================
/// Update Request Deserializations
/// ============================

bool update::requests::deserialization::deserializeMoveRequest(BufferReader& reader, MoveRequest& out_request)
{
    // read version
    uint8_t version{};
    if (!reader.read<uint8_t>(version) || version != ROBOT_CONTROL_MAJOR_VERSION)
    {
        return false; // version mismatch
    }

    // read request type
    uint8_t request_type_u8{};
    if (!reader.read<uint8_t>(request_type_u8))
    {
        return false; // parse error
    }
    out_request = static_cast<MoveRequest>(request_type_u8);
    return true;
}




/// ============================
/// Start Message Serialization
/// ============================

std::vector<std::byte>& status_messages::serialization::statusMessage(std::vector<std::byte>& buffer, uint64_t timestamp_us, Pose current_pose, Span<const double> joint_positions, RobotStatus status, const char* error_msg)
{
    buffer.clear();

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);
    ser::push<uint64_t>(buffer, timestamp_us);
    pushPose(buffer, current_pose);

    if (joint_positions.size() > start::requests::MAX_SUPPORTED_JOINTS)
    {
        throw std::invalid_argument("statusMessage: joint_positions size exceeds maximum supported joint count of 16");
    }

    ser::push<uint8_t>(buffer, static_cast<uint8_t>(joint_positions.size()));
    for (double joint_pos : joint_positions)
    {
        ser::push<double>(buffer, joint_pos);
    }
    ser::push<uint8_t>(buffer, static_cast<uint8_t>(status));

    bool has_error_msg = (status == RobotStatus::ERROR) && (error_msg != nullptr);
    ser::push<bool>(buffer, has_error_msg);
    if (has_error_msg)
    {
        uint64_t msg_len = static_cast<uint64_t>(std::strlen(error_msg));
        if (msg_len > start::requests::MAX_ERROR_MESSAGE_LENGTH)
        {
            msg_len = start::requests::MAX_ERROR_MESSAGE_LENGTH; // clip message length
        }
        ser::push<uint64_t>(buffer, msg_len);
        ser::pushBytes(buffer, error_msg, msg_len);
    }

    return buffer;
}


/// ============================
/// Status Message Deserialization
/// ============================

bool status_messages::deserialization::deserializeStatusMessage(BufferReader& reader, StatusMessage& out_status_message)
{
    // read version
    uint8_t version{};
    if (!reader.read<uint8_t>(version) || version != ROBOT_CONTROL_MAJOR_VERSION)
    {
        return false; // version mismatch
    }

    if (!reader.read<uint64_t>(out_status_message.timestamp_us))
    {
        return false;
    }

    if (!deserializePose(reader, out_status_message.current_pose))
    {
        return false;
    }

    uint8_t joint_count{};
    if (!reader.read<uint8_t>(joint_count) || joint_count > start::requests::MAX_SUPPORTED_JOINTS)
    {
        return false;
    }

    out_status_message.joint_positions.resize(joint_count);
    for (uint8_t i = 0; i < joint_count; ++i)
    {
        if (!reader.read<double>(out_status_message.joint_positions[i]))
        {
            return false;
        }
    }

    uint8_t status_u8{};
    if (!reader.read<uint8_t>(status_u8))
    {
        return false;
    }
    out_status_message.status = static_cast<RobotStatus>(status_u8);

    bool has_error_msg{};
    if (!reader.read<bool>(has_error_msg))
    {
        return false;
    }
    if (has_error_msg)
    {
        uint64_t msg_len{};
        if (!reader.read<uint64_t>(msg_len) || msg_len > start::requests::MAX_ERROR_MESSAGE_LENGTH)
        {
            return false;
        }
        out_status_message.error_msg.resize(msg_len);
        if (!reader.readBytes(out_status_message.error_msg.data(), msg_len))
        {
            return false;
        }
    }
    else
    {
        out_status_message.error_msg.clear();
    }

    return true;
}




/// ============================
/// Common Message Serialization
/// ============================

std::vector<std::byte>& common::serialization::errorMessage(std::vector<std::byte>& buffer, const char* error_msg)
{
    buffer.clear();

    ser::push<uint8_t>(buffer, ROBOT_CONTROL_MAJOR_VERSION);

    uint64_t msg_len = static_cast<uint64_t>(std::strlen(error_msg));
    if (msg_len > start::requests::MAX_ERROR_MESSAGE_LENGTH)
    {
        msg_len = start::requests::MAX_ERROR_MESSAGE_LENGTH; // clip message length
    }
    ser::push<uint64_t>(buffer, msg_len);
    ser::pushBytes(buffer, error_msg, msg_len);

    return buffer;
}



/// ============================
/// Common Message Deserialization
/// ============================

bool common::deserialization::deserializeErrorMessage(BufferReader& reader, std::string& out_message)
{
    // read version
    uint8_t version{};
    if (!reader.read<uint8_t>(version) || version != ROBOT_CONTROL_MAJOR_VERSION)
    {
        return false; // version mismatch
    }

    uint64_t msg_len{};
    if (!reader.read<uint64_t>(msg_len) || msg_len > start::requests::MAX_ERROR_MESSAGE_LENGTH)
    {
        return false;
    }
    out_message.resize(msg_len);
    if (!reader.readBytes(out_message.data(), msg_len))
    {
        return false;
    }

    return true;
}