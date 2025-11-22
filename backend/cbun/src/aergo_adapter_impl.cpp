#include "aergo/aergo_adapter_impl.h"
#include "aergo/logging.h"

#include <kr2_program_api/api_v1/bundles/arg_provider_xml.h>
#include <kr2_rc_api/api_v1/types.h>

using namespace aergo;


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

    // No specific activation parameters for now
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

    activation_parameters_.server_port = arg_provider.getInt(0);
    if (activation_parameters_.server_port < 1000 || activation_parameters_.server_port > 65535)
    {
        LOG_ERR("Invalid server port: " << activation_parameters_.server_port << ", must be between 1000 and 65535");
        return false;
    }

    return true;
}


CBUN_PCALL AergoConnector::Impl::onDeactivate()
{
    CBUN_PCALL_RET_OK
}


CBUN_PCALL AergoConnector::Impl::testMovement(const kr2_program_api::RobotPose &target)
{
    kr2rc_api::Pose target_pose { 
        .frame_ = target.toFrame(), 
        .ref_frame_id_ = kr2rc_api::Model::SysId::FRAME_WORLD 
    };

    kr2rc_api::Trajectory::CmdTrajectoryParams ws_tp;
    ws_tp.tracking_type_ = kr2rc_api::Trajectory::CmdTrajectoryParams::TT_WS_TARGET_SPEED;
    ws_tp.tracking_value_ = 0.1;
    ws_tp.blend_type_ = kr2rc_api::Trajectory::CmdTrajectoryParams::BT_WS_ACCELERATION;
    ws_tp.blend_value_ = 1;

    LOG_INFO("Starting movement to target pose...");
    base_->api_->rc_api_->arm_trajectory_->cmd_Follow(target_pose, &ws_tp);
    LOG_INFO("Movement finished.");


    // No hardware to move, so just return OK
    CBUN_PCALL_RET_OK
}