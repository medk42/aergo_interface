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


CBUN_PCALL AergoConnector::Impl::testMovement(const kr2_program_api::RobotPose &target)
{
    double r, p, y;
    target.rot().getRPY(r, p, y);
    const auto& pos = target.pos();

    kr2rc_api2::Pose target_pose { 
        .frame_ = {
            .M_ = kr2rc_api2::Rotation::RPY(r, p, y),
            .p_ = kr2rc_api2::Vector(pos.x().d(), pos.y().d(), pos.z().d())
        },
        .ref_frame_id_ = 0 
    };

    auto request = kr2rc_api2::Move::workSpaceBlend()
        .toTarget(target_pose)
        .withTargetType(kr2rc_api2::Move::TargetType::eStopPoint)
        .withTargetSpeed(0.400)
        .withBlendMaxAcceleration(8.000)
        .withSynchronization(kr2rc_api2::Move::SYNC);


    LOG_INFO("Starting movement to target pose...");
    request.follow();
    LOG_INFO("Movement finished.");


    // No hardware to move, so just return OK
    CBUN_PCALL_RET_OK
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

        while (rpc_server_->pollOnce(std::chrono::milliseconds(10))) ; // drain all messages

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

        auto start_us = micros();
        kr2rc_api2::Move::terminate().now(); // immediately stop the robot and clear its trajectory
        LOG_INFO("RobotControl CancelMovement processed in " << (micros() - start_us) << " us.");

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
        const auto& move_request = std::get<start::requests::deserialization::MoveJointRequest>(request_variant);

        if (move_request.joint_targets.size() != robot_specs.num_joints)
        {
            return errorResponse(out_response_blob, "Invalid joint count in MoveJoint request.");
        }

        if (move_request.acceleration > robot_specs.max_acceleration_angular ||
            move_request.speed > robot_specs.max_velocity_angular)
        {
            return errorResponse(out_response_blob, "Requested speed or acceleration exceeds robot limits.");
        }

        finishCurrentMoveAction(false, "Move was interrupted by a new move request.");

        kr2rc_api2::Move::jointSpaceBlend()
            .toTarget(kr2rc_api2::JSVector(move_request.joint_targets.data()))
            .withTargetType(kr2rc_api2::Move::TargetType::eStopPoint)
            .withTargetSpeed(move_request.speed)
            .withBlendMaxAcceleration(move_request.acceleration)
            .withSynchronization(kr2rc_api2::Move::ASYNC)
            .follow();

        current_move_action_id_ = generateNextActionId();

        return Response {
            .resp_type = RespType::SUCCESS_IN_PROGRESS,
            .action_id = *current_move_action_id_
        };
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

        rpc_server_->sendFinishedMessage(ri::FinishedMessage {
            .action_id = *current_move_action_id_,
            .success = success
        }, Span<const std::byte>(response_blob_buffer_.data(), response_blob_buffer_.size()));

        current_move_action_id_.reset();
    }
}


void AergoConnector::Impl::handleUpdates()
{
    using namespace ri;
    using namespace robot_control;


    uint64_t timestamp_us = static_cast<uint64_t>(micros());

    auto [robot_status, error_message] = readRobotStatus();
    auto robot_pose = readRobotPosition();

    auto joint_positions = base_->api_->rc_api_->arm_model_->read_ModelJointsByDuid(kr2rc_api::Model::SysId::JOINT_POSITIONS);
    Span<const double> joint_positions_span(joint_positions->values_.data(), robot_specs.num_joints);


    status_messages::serialization::statusMessage(
        response_blob_buffer_, timestamp_us, robot_pose, joint_positions_span, robot_status, error_message
    );

    rpc_server_->sendStatusMessage(
        rpc::StatusMessage { .feature = RobotFeature::ROBOT_CONTROL }, 
        Span<const std::byte>(response_blob_buffer_.data(), response_blob_buffer_.size())
    );

    LOG_INFO("Sending RobotControl status: status=" << 
        (robot_status == RobotStatus::IDLE ? "IDLE" : 
         robot_status == RobotStatus::MOVING ? "MOVING" : 
         "ERROR") << ", msg: " << (error_message ? error_message : "NO_MSG") << ", timestamp_us=" << timestamp_us << 
        ", pose=(" << robot_pose.position.x << ", " << robot_pose.position.y << ", " << robot_pose.position.z << ")"   

    );



    if (robot_status == RobotStatus::IDLE && current_move_action_id_)
    {
        finishCurrentMoveAction(true);
    }
    else if (robot_status == RobotStatus::ERROR && current_move_action_id_)
    {
        finishCurrentMoveAction(false, error_message);
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
        kr2rc_api::State::SysStat_RCState::MOTION_FLAG_STANDBY |
        kr2rc_api::State::SysStat_RCState::MOTION_FLAG_TRACKING
    );



    RobotStatus current_status = RobotStatus::IDLE;
    const char* error_message = nullptr;

    if ((safety_flags & (kr2rc_api::State::SysStat_RCState::SAFETY_FLAG_PSTOP | kr2rc_api::State::SysStat_RCState::SAFETY_FLAG_ESTOP)) != 0)
    {
        current_status = RobotStatus::ERROR;
        error_message = "Safety flags are engaged (PSTOP, ESTOP).";
    }
    else if (motion_flags_cleared != 0)
    {
        current_status = RobotStatus::ERROR;
        error_message = "Motion flags indicate an unexpected state (not IDLE or MOVING).";
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


ri::robot_control::Pose AergoConnector::Impl::readRobotPosition()
{
    using namespace ri;
    using namespace robot_control;


    const kr2rc_api::Model::TF *tf = base_->api_->rc_api_->arm_model_->read_TransformationByDuid(kr2rc_api::Model::SysId::FRAME_ROBOTX_TCP, kr2rc_api::Model::SysId::FRAME_WORLD);

    Pose current_pose {
        .position = {
            .x = tf->pose_.p_.x(),
            .y = tf->pose_.p_.y(),
            .z = tf->pose_.p_.z()
        },
        .orientation = {}
    };

    tf->pose_.M_.getQuaternion(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    );

    return current_pose;
}