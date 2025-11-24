#include "aergo/aergo_adapter_impl.h"
#include "aergo/logging.h"

#include <kr2_program_api/api_v1/bundles/arg_provider_xml.h>
#include <kr2_rc_api/api_v2/trajectory_moves.h>

using namespace aergo;


void CbunLogger::log(aergo::robot::kassow::rpc::RpcLogType type, const char* message) const noexcept
{
    switch (type)
    {
        case aergo::robot::kassow::rpc::RpcLogType::INFO:
            LOG_INFO("[RPC] " << message);
            break;
        case aergo::robot::kassow::rpc::RpcLogType::WARNING:
            LOG_WARN("[RPC] " << message);
            break;
        case aergo::robot::kassow::rpc::RpcLogType::ERROR:
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

    rpc_server_ = std::make_unique<aergo::robot::kassow::rpc::RpcServer>(&cbun_logger_);
    // TODO setup callbacks and background thread
    if (!rpc_server_->start(activation_parameters_.server_port))
    {
        rpc_server_.reset();
        LOG_ERR("Failed to start RPC server on port " << activation_parameters_.server_port);
        CBUN_PCALL_RET_ERROR(-2, "Failed to start RPC server.");
    }

    CBUN_PCALL_RET_OK
}


CBUN_PCALL AergoConnector::Impl::onDeactivate()
{
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