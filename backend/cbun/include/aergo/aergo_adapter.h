#pragma once

#define EXPORT __attribute__((visibility("default")))
#define HIDDEN __attribute__((visibility("hidden")))

#include <kr2_program_api/api_v1/bundles/custom_device.h>

namespace aergo {

    class EXPORT AergoConnector : public kr2_bundle_api::CustomDevice {
    public:
        
        AergoConnector(boost::shared_ptr<kr2_program_api::ProgramInterface> api, const boost::property_tree::ptree &xml_bundle_node);
        ~AergoConnector();

        virtual int onCreate();
        virtual int onDestroy();
        virtual int onBind();
        virtual int onUnbind();
        
        CBUN_PCALL testMovement(const kr2_program_api::RobotPose &target);

    protected:
        virtual CBUN_PCALL onActivate(const boost::property_tree::ptree &param_tree);
        virtual CBUN_PCALL onDeactivate();
        virtual CBUN_PCALL onMount(const boost::property_tree::ptree &param_tree);
        virtual CBUN_PCALL onUnmount();
    
    private:
        class HIDDEN Impl;
        std::unique_ptr<Impl> impl_;
    };
}