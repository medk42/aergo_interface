
#include "aergo/aergo_adapter.h"
#include "aergo/aergo_adapter_impl.h"

using namespace aergo;

REGISTER_CLASS(aergo::AergoConnector)



AergoConnector::AergoConnector(boost::shared_ptr<kr2_program_api::ProgramInterface> api, const boost::property_tree::ptree &xml_bundle_node)
: CustomDevice(api, xml_bundle_node), impl_(std::make_unique<Impl>(this)) {}

AergoConnector::~AergoConnector() = default;

int AergoConnector::onCreate() { return impl_->onCreate(); }
int AergoConnector::onDestroy() { return impl_->onDestroy(); }
int AergoConnector::onBind() { return impl_->onBind(); }
int AergoConnector::onUnbind() { return impl_->onUnbind(); }

CBUN_PCALL AergoConnector::testMovement(const kr2_program_api::RobotPose &target) { return impl_->testMovement(target); }

CBUN_PCALL AergoConnector::onActivate(const boost::property_tree::ptree &param_tree) { return impl_->onActivate(param_tree); }
CBUN_PCALL AergoConnector::onDeactivate() { return impl_->onDeactivate(); }
CBUN_PCALL AergoConnector::onMount(const boost::property_tree::ptree &param_tree) { return impl_->onMount(param_tree); }
CBUN_PCALL AergoConnector::onUnmount() { return impl_->onUnmount(); }