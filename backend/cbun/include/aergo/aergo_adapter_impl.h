#pragma once

#include "aergo/aergo_adapter.h"

namespace aergo
{
    class AergoConnector::Impl {
    public:
        
        Impl(AergoConnector* base) : base_(base) {}
        ~Impl() = default;

        virtual int onCreate();
        virtual int onDestroy();

        // only acts as a server, no instance code needed
        virtual int onBind() { return 0; }   
        virtual int onUnbind() { return 0; }
        
        CBUN_PCALL testMovement(const kr2_program_api::RobotPose &target);

        virtual CBUN_PCALL onActivate(const boost::property_tree::ptree &param_tree);
        virtual CBUN_PCALL onDeactivate();

        // There is no hardware to mount/unmount, so these are no-ops
        virtual CBUN_PCALL onMount(const boost::property_tree::ptree &param_tree) { CBUN_PCALL_RET_OK; }
        virtual CBUN_PCALL onUnmount() { CBUN_PCALL_RET_OK; }

    private:
        bool processActivationParams(const boost::property_tree::ptree &tree);

        AergoConnector* base_;

        struct {
            int server_port;
        } activation_parameters_;
    };
}