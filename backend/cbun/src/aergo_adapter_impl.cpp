#include "aergo/aergo_adapter_impl.h"
#include "aergo/logging.h"
#include "module_helpers/serialization_helper/serialization_helper.h"

#include <kr2_program_api/api_v1/bundles/arg_provider_xml.h>
#include <kr2_rc_api/api_v2/trajectory_moves.h>

#include <chrono>

using namespace aergo;





double degToRad(double degrees)
{
    return degrees * 3.14159265358979323846 / 180.0;
}


double mmToM(double mm)
{
    return mm / 1000.0;
}


ri::robot_control::RobotSpecs robot_specs {
    .max_velocity_linear = mmToM(2000),          // m/s
    .max_velocity_angular = degToRad(225),       // rad/s
    .max_acceleration_linear = mmToM(8000),      // m/s²
    .max_acceleration_angular = degToRad(360),   // rad/s²
    .num_joints = 7,
    .joint_limits = {                            // in radians
        { degToRad(-360), degToRad(360) },
        { degToRad(-70), degToRad(180) },
        { degToRad(-360), degToRad(360) },
        { degToRad(-70), degToRad(180) },
        { degToRad(-360), degToRad(360) },
        { degToRad(-360), degToRad(360) },
        { degToRad(-360), degToRad(360) },
    }
};


ri::Response errorResponse(std::vector<std::byte>& out_response_blob, const char* error_message = nullptr)
{
    if (error_message)
    {
        ri::robot_control::common::serialization::errorMessage(
            out_response_blob, error_message
        );
    }

    return ri::Response {
        .resp_type = ri::RespType::FAILURE,
        .action_id = 0
    };
}





void CbunLogger::log(rpc::RpcLogType type, const char* message) const noexcept
{
    switch (type)
    {
        case rpc::RpcLogType::INFO:
            LOG_INFO("[RPC] " << message);
            break;
        case rpc::RpcLogType::WARNING:
            LOG_WARN("[RPC] " << message);
            break;
        case rpc::RpcLogType::ERROR:
            LOG_ERR("[RPC] " << message);
            break;
    }
}


int AergoConnector::Impl::onCreate()
{

    if (base_->activation_tree_)
    {
        kr2_program_api::CmdResult<> result = base_->activate(*base_->activation_tree_);
        switch (result.result_)
        {
            case kr2_program_api::CmdResult<>::EXCEPTION:
                base_->publish<kr2_bundle::Exception>(kr2_bundle::Exception(base_->getCBun(), base_->getClass(), base_->getLabel(), result.code_, result.message_));
                break;
            case kr2_program_api::CmdResult<>::ERROR:
                base_->publish<kr2_bundle::Error>(kr2_bundle::Error(base_->getCBun(), base_->getClass(), base_->getLabel(), result.code_, result.message_));
                break;
        }
    }

    return 0;
}


int AergoConnector::Impl::onDestroy()
{
    onDeactivate();

    return 0;
}


CBUN_PCALL AergoConnector::Impl::onActivate(const boost::property_tree::ptree &param_tree)
{
    if (!processActivationParams(param_tree)) {
        LOG_ERR("Invalid activation params");
        CBUN_PCALL_RET_ERROR(-1, "Invalid activation parameters.");
    }

    rpc_server_ = std::make_unique<rpc::RpcServer>(&cbun_logger_);
    rpc_server_->setRequestHandler([this](const rpc::RpcServer::IncomingRequest& request) {
        processRequest(request);
    });

    if (!rpc_server_->start(activation_parameters_.server_port))
    {
        rpc_server_.reset();
        LOG_ERR("Failed to start RPC server on port " << activation_parameters_.server_port);
        CBUN_PCALL_RET_ERROR(-2, "Failed to start RPC server.");
    }

    rpc_server_running_ = true;
    rpc_server_thread_ = std::thread(&AergoConnector::Impl::rpcServerThreadFunc, this);

    CBUN_PCALL_RET_OK
}


CBUN_PCALL AergoConnector::Impl::onDeactivate()
{
    rpc_server_running_ = false;
    if (rpc_server_thread_.joinable())
    {
        rpc_server_thread_.join();
    }

    if (rpc_server_)
    {
        rpc_server_->stop();
    }
    rpc_server_.reset();

    CBUN_PCALL_RET_OK
}


bool AergoConnector::Impl::processActivationParams(const boost::property_tree::ptree &tree)
{
    kr2_bundle_api::ArgProviderXml arg_provider(tree);

    const int EXPECTED_PARAMS = 1;
    if (arg_provider.getArgCount() != EXPECTED_PARAMS) {
        LOG_ERR("Unexpected param count: actual=" << arg_provider.getArgCount() << ", expected=" << EXPECTED_PARAMS);
        return false;
    }

    int port = arg_provider.getInt(0);
    if (port < 1 || port > 65535)
    {
        LOG_ERR("Invalid server port: " << port << ", must be between 1 and 65535");
        return false;
    }
    activation_parameters_.server_port = static_cast<u_int16_t>(port);

    return true;
}


int64_t AergoConnector::Impl::micros() const noexcept
{
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}


void AergoConnector::Impl::rpcServerThreadFunc()
{
    while (rpc_server_running_)
    {
        base_->api_->rc_api_->spin(); // update internal state

        while (rpc_server_->pollOnce(std::chrono::milliseconds(200))) ; // drain all messages

        handleUpdates();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


// called synchronously from RPC server thread from the pollOnce method
void AergoConnector::Impl::processRequest(const rpc::RpcServer::IncomingRequest& request)
{
    if (!rpc_server_)
    {
        LOG_WARN("Received RPC request while server is not active.");
        return;
    }

    ri::Response response;
    response_blob_buffer_.clear();

    switch (request.request.feature)
    {
        case ri::RobotFeature::ROBOT_CONTROL:
            response = processRequestRobotControl(request.request.req_type, request.request.action_id, request.blob, response_blob_buffer_);
            break;

        default:
            response = ri::Response {
                .resp_type = ri::RespType::FEATURE_NOT_SUPPORTED,
                .action_id = request.request.action_id
            };
            LOG_WARN("Received request for unsupported feature: " << static_cast<uint64_t>(request.request.feature));
            break;
    }

    rpc_server_->sendResponse(request.request_id, response, Span<const std::byte>(response_blob_buffer_.data(), response_blob_buffer_.size()));    
}


const char* parseCmdResult(const kr2rc_api2::CmdResult& res)
{
    switch (res.err_code_)
    {
        case kr2rc_api2::Move::CmdResultCodes::eAccepted:
            return "Move request was passed to the robot controller.";
        case kr2rc_api2::Move::CmdResultCodes::eInvalidVariables:
            return "A parameter in the move request has an invalid value, e.g., a Pose containing NANs.";
        case kr2rc_api2::Move::CmdResultCodes::eMissingParameters:
            return "An obligatory parameter was not set in the move request.";
        case kr2rc_api2::Move::CmdResultCodes::eConflictingParameters:
            return "Mutually exclusive parameters were detected in the move request.";
        case kr2rc_api2::Move::CmdResultCodes::eUnknownError:
        default:
            return "An unknown error occurred while processing the move request.";
        
    }
}


ri::Response AergoConnector::Impl::processRequestRobotControl(
    ri::ReqType req_type, 
    uint64_t action_id, 
    Span<const std::byte> request_blob, 
    std::vector<std::byte>& out_response_blob
)
{
    using namespace ri;
    using namespace robot_control;

    start::requests::deserialization::BufferReader reader(
        reinterpret_cast<const void*>(request_blob.data()), 
        request_blob.size()
    );

    if (req_type == ReqType::START_ACTION)
    {
        start::requests::deserialization::RequestVariant request_variant;
        if (!start::requests::deserialization::deserialize(reader, request_variant))
        {
            LOG_WARN("Failed to deserialize RobotControl START_ACTION request");
            return Response {
                .resp_type = RespType::DATA_INVALID,
                .action_id = action_id
            };
        }

        return processStartRequestRobotControl(request_variant, out_response_blob);
    }
    else if (req_type == ReqType::UPDATE_ACTION)
    {
        update::requests::MoveRequest update_move_request;
        if (!update::requests::deserialization::deserializeMoveRequest(reader, update_move_request))
        {
            LOG_WARN("Failed to deserialize RobotControl UPDATE_ACTION request");
            return Response {
                .resp_type = RespType::DATA_INVALID,
                .action_id = action_id
            };
        }

        if (update_move_request != update::requests::MoveRequest::CancelMovement)
        {
            LOG_WARN("Received unsupported RobotControl UPDATE_ACTION request variant.");
            return errorResponse(out_response_blob, "Unsupported update action request, expected CancelMovement.");
        }

        if (!current_move_action_id_ || *current_move_action_id_ != action_id)
        {
            LOG_WARN("Received RobotControl UPDATE_ACTION request for unknown action_id: " << action_id);
            return Response {
                .resp_type = RespType::NOT_IN_PROGRESS,
                .action_id = action_id
            };
        }

        kr2rc_api2::CmdResult res = kr2rc_api2::Move::terminate().now(); // immediately stop the robot and clear its trajectory
        if (res.err_code_ != kr2rc_api2::Move::CmdResultCodes::eAccepted)
        {
            return errorResponse(out_response_blob, parseCmdResult(res));
        }

        return Response {
            .resp_type = RespType::SUCCESS,
            .action_id = action_id
        };   
    }
    else
    {
        LOG_WARN("Received RobotControl request with unknown ReqType: " << static_cast<uint8_t>(req_type));
        return Response {
            .resp_type = RespType::DATA_INVALID,
            .action_id = action_id
        };
    }
}


ri::Response AergoConnector::Impl::processStartRequestRobotControl(
    const ri::robot_control::start::requests::deserialization::RequestVariant& request_variant, 
    std::vector<std::byte>& out_response_blob
)
{
    using namespace ri;
    using namespace robot_control;

    if (std::holds_alternative<start::requests::deserialization::GetRobotSpecsRequest>(request_variant))
    {
        start::responses::serialization::robotSpecs(out_response_blob, robot_specs);

        return Response {
            .resp_type = RespType::SUCCESS,
            .action_id = 0
        };
    }
    else if (std::holds_alternative<start::requests::deserialization::MoveJointRequest>(request_variant))
    {
        return processMoveJoint(
            std::get<start::requests::deserialization::MoveJointRequest>(request_variant),
            out_response_blob
        );
    }
    else if (std::holds_alternative<start::requests::deserialization::MoveLinearRequest>(request_variant))
    {
        return processMoveLinear(
            std::get<start::requests::deserialization::MoveLinearRequest>(request_variant),
            out_response_blob
        );
    }
    else if (std::holds_alternative<start::requests::deserialization::MoveArcRequest>(request_variant))
    {
        return processMoveArc(
            std::get<start::requests::deserialization::MoveArcRequest>(request_variant),
            out_response_blob
        );
    }
    else if (std::holds_alternative<start::requests::deserialization::MoveTrajectoryRequest>(request_variant))
    {
        return processMoveTrajectory(
            std::get<start::requests::deserialization::MoveTrajectoryRequest>(request_variant),
            out_response_blob
        );
    }
    else
    {
        LOG_WARN("Received unsupported RobotControl START_ACTION request variant.");
        return Response {
            .resp_type = RespType::DATA_INVALID,
            .action_id = 0
        };
    }
}


ri::Response AergoConnector::Impl::processMoveJoint(
    const rc::start::requests::deserialization::MoveJointRequest& move_joint_request,
    std::vector<std::byte>& out_response_blob
)
{
    using namespace ri;
    using namespace rc;

    if (move_joint_request.joint_targets.size() != robot_specs.num_joints)
    {
        return errorResponse(out_response_blob, "Invalid joint count in MoveJoint request.");
    }

    if (move_joint_request.acceleration > robot_specs.max_acceleration_angular ||
        move_joint_request.speed > robot_specs.max_velocity_angular)
    {
        return errorResponse(out_response_blob, "Requested speed or acceleration exceeds robot limits.");
    }

    finishCurrentMoveAction(false, "Move was interrupted by a new move request.");

    kr2rc_api2::CmdResult res = kr2rc_api2::Move::jointSpaceBlend()
        .toTarget(kr2rc_api2::JSVector(move_joint_request.joint_targets.data()))
        .withTargetType(kr2rc_api2::Move::TargetType::eStopPoint)
        .withTargetSpeed(move_joint_request.speed)
        .withBlendMaxAcceleration(move_joint_request.acceleration)
        .withSynchronization(kr2rc_api2::Move::ASYNC)
        .follow();

    if (res.err_code_ != kr2rc_api2::Move::CmdResultCodes::eAccepted)
    {
        return errorResponse(out_response_blob, parseCmdResult(res));
    }

    current_move_action_id_ = generateNextActionId();
    current_move_start_time_us_ = micros();

    return Response {
        .resp_type = RespType::SUCCESS_IN_PROGRESS,
        .action_id = *current_move_action_id_
    };
}


kr2rc_api2::Pose kr2PoseFromRcPose(const rc::Pose& rc_pose)
{
    return kr2rc_api2::Pose {
        .frame_ = {
            .M_ = kr2rc_api2::Rotation::Quaternion(
                rc_pose.orientation.x, 
                rc_pose.orientation.y, 
                rc_pose.orientation.z, 
                rc_pose.orientation.w
            ),
            .p_ = kr2rc_api2::Vector(
                rc_pose.position.x, 
                rc_pose.position.y, 
                rc_pose.position.z
            )
        },
        .ref_frame_id_ = 0
    };
}


ri::Response AergoConnector::Impl::processMoveLinear(
    const rc::start::requests::deserialization::MoveLinearRequest& move_linear_request,
    std::vector<std::byte>& out_response_blob
)
{
    using namespace ri;
    using namespace rc;

    if (move_linear_request.speed > robot_specs.max_velocity_linear ||
        move_linear_request.acceleration > robot_specs.max_acceleration_linear)
    {
        return errorResponse(out_response_blob, "Requested speed or acceleration exceeds robot limits.");
    }

    finishCurrentMoveAction(false, "Move was interrupted by a new move request.");

    kr2rc_api2::CmdResult res = kr2rc_api2::Move::workSpaceBlend()
        .toTarget(kr2PoseFromRcPose(move_linear_request.pose_target))
        .withTargetType(kr2rc_api2::Move::TargetType::eStopPoint)
        .withTargetSpeed(move_linear_request.speed)
        .withBlendMaxAcceleration(move_linear_request.acceleration)
        .withSynchronization(kr2rc_api2::Move::ASYNC)
        .follow();

    if (res.err_code_ != kr2rc_api2::Move::CmdResultCodes::eAccepted)
    {
        return errorResponse(out_response_blob, parseCmdResult(res));
    }

    current_move_action_id_ = generateNextActionId();
    current_move_start_time_us_ = micros();

    return Response {
        .resp_type = RespType::SUCCESS_IN_PROGRESS,
        .action_id = *current_move_action_id_
    };
}


ri::Response AergoConnector::Impl::processMoveArc(
    const rc::start::requests::deserialization::MoveArcRequest& move_arc_request,
    std::vector<std::byte>& out_response_blob
)
{
    using namespace ri;
    using namespace rc;

    if (move_arc_request.speed > robot_specs.max_velocity_linear ||
        move_arc_request.acceleration > robot_specs.max_acceleration_linear)
    {
        return errorResponse(out_response_blob, "Requested speed or acceleration exceeds robot limits.");
    }

    if (move_arc_request.as_circle && move_arc_request.circle_percentage <= 0.0)
    {
        return errorResponse(out_response_blob, "Invalid circle percentage in MoveArc request.");
    }

    finishCurrentMoveAction(false, "Move was interrupted by a new move request.");

    auto arc_req = kr2rc_api2::Move::arc()
        .withGeometry(
            kr2PoseFromRcPose(move_arc_request.pose_through), 
            kr2PoseFromRcPose(move_arc_request.pose_target)
        )
        .withTargetType(kr2rc_api2::Move::TargetType::eStopPoint)
        .withConstantSpeed(move_arc_request.speed)
        .withAcceleration(move_arc_request.acceleration)
        .withOrientation(
            move_arc_request.orientation_type == OrientationType::FIXED ? 
                kr2rc_api2::Move::TrajectoryArcRequest::Orientation::eFixed : 
                kr2rc_api2::Move::TrajectoryArcRequest::Orientation::eTangential
        )
        .withSynchronization(kr2rc_api2::Move::ASYNC);
    
    if (move_arc_request.as_circle)
    {
        arc_req = arc_req.asCircle(move_arc_request.circle_percentage * 2.0 * 3.14159265358979323846);
    }
    
    kr2rc_api2::CmdResult res = arc_req.follow();
    
    if (res.err_code_ != kr2rc_api2::Move::CmdResultCodes::eAccepted)
    {
        return errorResponse(out_response_blob, parseCmdResult(res));
    }

    current_move_action_id_ = generateNextActionId();
    current_move_start_time_us_ = micros();

    return Response {
        .resp_type = RespType::SUCCESS_IN_PROGRESS,
        .action_id = *current_move_action_id_
    };
}


ri::Response AergoConnector::Impl::processMoveTrajectory(
    const rc::start::requests::deserialization::MoveTrajectoryRequest& move_trajectory_request,
    std::vector<std::byte>& out_response_blob
)
{
    using namespace ri;
    using namespace rc;

    if (move_trajectory_request.speed > robot_specs.max_velocity_linear ||
        move_trajectory_request.acceleration > robot_specs.max_acceleration_linear)
    {
        return errorResponse(out_response_blob, "Requested speed or acceleration exceeds robot limits.");
    }

    if (move_trajectory_request.pose_targets.size() < 2)
    {
        return errorResponse(out_response_blob, "MoveTrajectory request must contain at least 2 target poses.");
    }

    finishCurrentMoveAction(false, "Move was interrupted by a new move request.");

    for (size_t i = 0; i < move_trajectory_request.pose_targets.size(); ++i)
    {
        auto spline_move = kr2rc_api2::Move::spline()
            .withKnotPoint(kr2PoseFromRcPose(move_trajectory_request.pose_targets[i]))
            .withApproximateSpeed(move_trajectory_request.speed)
            .withAcceleration(move_trajectory_request.acceleration)
            .withSynchronization(kr2rc_api2::Move::ASYNC)
            .withOrientation(
                move_trajectory_request.orientation_type == OrientationType::FIXED ? 
                    kr2rc_api2::Move::TrajectorySplineRequest::Orientation::eFixed : 
                    kr2rc_api2::Move::TrajectorySplineRequest::Orientation::eTangentialSecondaryZ
            );
        
        if (i == move_trajectory_request.pose_targets.size() - 1)
        {
            spline_move = spline_move.withTargetType(kr2rc_api2::Move::TargetType::eStopPoint);
        }
        else
        {
            spline_move = spline_move.withTargetType(kr2rc_api2::Move::TargetType::eViaPoint);
        }

        kr2rc_api2::CmdResult res;
        if (i == 0)
        {
            res = spline_move.follow();
        }
        else
        {
            res = spline_move.add();
        }

        if (res.err_code_ != kr2rc_api2::Move::CmdResultCodes::eAccepted)
        {
            return errorResponse(out_response_blob, parseCmdResult(res));
        }
    }

    current_move_action_id_ = generateNextActionId();
    current_move_start_time_us_ = micros();

    return Response {
        .resp_type = RespType::SUCCESS_IN_PROGRESS,
        .action_id = *current_move_action_id_
    };
}


void AergoConnector::Impl::finishCurrentMoveAction(bool success, const char* error_message)
{
    if (current_move_action_id_)
    {
        if (error_message)
        {
            ri::robot_control::common::serialization::errorMessage(
                response_blob_buffer_, error_message
            );
        }
        else
        {
            response_blob_buffer_.clear();
        }

        if (
            !rpc_server_->sendFinishedMessage(ri::FinishedMessage {
                .action_id = *current_move_action_id_,
                .success = success
            }, Span<const std::byte>(response_blob_buffer_.data(), response_blob_buffer_.size()))
        )
        {
            LOG_WARN("Failed to send FinishedMessage for move action " << *current_move_action_id_);
        }

        current_move_action_id_.reset();
    }
}


void AergoConnector::Impl::handleUpdates()
{
    using namespace ri;
    using namespace robot_control;


    uint64_t timestamp_us = static_cast<uint64_t>(micros());

    auto [robot_status, error_message] = readRobotStatus();

    Pose base_pose, flange_pose, end_effector_pose;
    if (!readRobotPosition(base_pose, flange_pose, end_effector_pose))
    {
        LOG_ERR("Failed to read robot position for status update.");
        return;
    }

    auto joint_positions = base_->api_->rc_api_->arm_model_->read_ModelJointsByDuid(kr2rc_api::Model::SysId::JOINT_POSITIONS);
    Span<const double> joint_positions_span(joint_positions->values_.data(), robot_specs.num_joints);


    status_messages::serialization::statusMessage(
        response_blob_buffer_, timestamp_us, 
        base_pose, flange_pose, end_effector_pose, 
        joint_positions_span, robot_status, error_message
    );

    rpc_server_->sendStatusMessage(
        rpc::StatusMessage { .feature = RobotFeature::ROBOT_CONTROL }, 
        Span<const std::byte>(response_blob_buffer_.data(), response_blob_buffer_.size())
    );


    if (micros() > current_move_start_time_us_ + min_move_duration_us_)
    {
        if (robot_status == RobotStatus::IDLE && current_move_action_id_)
        {
            finishCurrentMoveAction(true);
        }
        else if (robot_status == RobotStatus::ERROR && current_move_action_id_)
        {
            finishCurrentMoveAction(false, error_message);
        }
    }
}


std::tuple<ri::robot_control::RobotStatus, const char*> AergoConnector::Impl::readRobotStatus()
{
    using namespace ri;
    using namespace robot_control;


    auto system_state = base_->api_->rc_api_->system_state_->read_SysStat_RCState();
    auto motion_flags = system_state->unsignedValue(kr2rc_api::State::SysStat_RCState::UITM_MOTION_FLAGS);
    auto safety_flags = system_state->unsignedValue(kr2rc_api::State::SysStat_RCState::UITM_SAFETY_FLAGS);

    bool is_standby = (motion_flags & kr2rc_api::State::SysStat_RCState::MOTION_FLAG_STANDBY) == kr2rc_api::State::SysStat_RCState::MOTION_FLAG_STANDBY;
    bool is_tracking = (motion_flags & kr2rc_api::State::SysStat_RCState::MOTION_FLAG_TRACKING) == kr2rc_api::State::SysStat_RCState::MOTION_FLAG_TRACKING;

    // In our use, the robot should only enter standby or tracking mode when it is not actively moving via program control,
    // in freedrive mode or some error state.
    auto motion_flags_cleared = motion_flags & ~(
        kr2rc_api::State::SysStat_RCState::MOTION_FLAG_STANDBY  |
        kr2rc_api::State::SysStat_RCState::MOTION_FLAG_TRACKING | 
        kr2rc_api::State::SysStat_RCState::MOTION_FLAG_SYNC         // SYNC happens at the end of a move, we can ignore it
    );



    RobotStatus current_status = RobotStatus::IDLE;
    const char* error_message = nullptr;

    if ((safety_flags & (kr2rc_api::State::SysStat_RCState::SAFETY_FLAG_PSTOP | kr2rc_api::State::SysStat_RCState::SAFETY_FLAG_ESTOP)) != 0)
    {
        current_status = RobotStatus::ERROR;
        error_message = "Safety flags are engaged (PSTOP, ESTOP).";
        LOG_WARN("Robot in ERROR state due to safety flags: " << safety_flags);
    }
    else if (motion_flags_cleared != 0)
    {
        current_status = RobotStatus::ERROR;
        error_message = "Motion flags indicate an unexpected state (not IDLE or MOVING).";
        LOG_WARN("Robot in ERROR state due to unexpected motion flags: " << motion_flags);
    }
    else if (is_tracking)
    {
        current_status = RobotStatus::MOVING;
    }
    else if (is_standby)
    {
        current_status = RobotStatus::IDLE;
    }
    else
    {
        current_status = RobotStatus::ERROR;
        error_message = "Unknown motion state.";
    }


    return { current_status, error_message };
}


ri::robot_control::Pose parseRobotPoseFromTF(const kr2rc_api::Model::TF& tf)
{
    using namespace ri;
    using namespace robot_control;

    Pose current_pose {
        .position = {
            .x = tf.pose_.p_.x(),
            .y = tf.pose_.p_.y(),
            .z = tf.pose_.p_.z()
        },
        .orientation = {}
    };

    tf.pose_.M_.getQuaternion(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    );

    return current_pose;
}


bool AergoConnector::Impl::readRobotPosition(
    ri::robot_control::Pose& out_base_pose, 
    ri::robot_control::Pose& out_flange_pose,
    ri::robot_control::Pose& out_end_effector_pose
)
{
    using namespace ri;
    using namespace robot_control;


    const kr2rc_api::Model::TF *tcp = base_->api_->rc_api_->arm_model_->read_TransformationByDuid(kr2rc_api::Model::SysId::FRAME_ROBOTX_TCP, kr2rc_api::Model::SysId::FRAME_WORLD);
    const kr2rc_api::Model::TF *tfc = base_->api_->rc_api_->arm_model_->read_TransformationByDuid(kr2rc_api::Model::SysId::FRAME_ROBOT_FLANGE, kr2rc_api::Model::SysId::FRAME_WORLD);
    const kr2rc_api::Model::TF *base = base_->api_->rc_api_->arm_model_->read_TransformationByDuid(kr2rc_api::Model::SysId::FRAME_ROBOT_BASE, kr2rc_api::Model::SysId::FRAME_WORLD);

    if (!tcp || !tfc || !base)
    {
        LOG_ERR("Reading robot transformations to FRAME_WORLD returned nullptr!");
        return false;
    }

    out_end_effector_pose = parseRobotPoseFromTF(*tcp);
    out_flange_pose = parseRobotPoseFromTF(*tfc);
    out_base_pose = parseRobotPoseFromTF(*base);

    return true;
}