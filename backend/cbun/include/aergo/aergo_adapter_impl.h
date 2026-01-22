#pragma once

#include "aergo/aergo_adapter.h"
#include "robot_module_kassow/rpc/rpc_transport.h"
#include "module_helpers/robot_interface/cpp17_utils.h"
#include "module_helpers/robot_interface/features/robot_control/messages.h"

#include <memory>
#include <vector>
#include <cstddef>
#include <thread>
#include <atomic>
#include <tuple>

namespace aergo
{
    namespace rpc = aergo::robot::kassow::rpc;
    namespace ri = aergo::module::helpers::robot_interface;
    namespace rc = ri::robot_control;

    class CbunLogger : public rpc::RpcLogger
    {
    public:
        virtual void log(rpc::RpcLogType type, const char* message) const noexcept override;
    };

    class AergoConnector::Impl {
    public:
        
        Impl(AergoConnector* base) : base_(base) {}
        ~Impl();

        virtual int onCreate();
        virtual int onDestroy();

        // only acts as a server, no instance code needed
        virtual int onBind() { return 0; }   
        virtual int onUnbind() { return 0; }

        virtual CBUN_PCALL onActivate(const boost::property_tree::ptree &param_tree);
        virtual CBUN_PCALL onDeactivate();

        // There is no hardware to mount/unmount, so these are no-ops
        virtual CBUN_PCALL onMount(const boost::property_tree::ptree &param_tree) { CBUN_PCALL_RET_OK; }
        virtual CBUN_PCALL onUnmount() { CBUN_PCALL_RET_OK; }

    private:
        bool processActivationParams(const boost::property_tree::ptree &tree);

        int64_t micros() const noexcept;

        void stopTrajectoryWorker();

        void rpcServerThreadFunc();
        uint64_t generateNextActionId() { return next_action_id_++; }
        void finishCurrentMoveAction(bool success, const char* error_message = nullptr); // send finished message for current move action if any

        void processRequest(const rpc::RpcServer::IncomingRequest& request);
        ri::Response processRequestRobotControl(ri::ReqType req_type, uint64_t action_id, Span<const std::byte> request_blob, std::vector<std::byte>& out_response_blob);
        ri::Response processStartRequestRobotControl(
            const rc::start::requests::deserialization::RequestVariant& request_variant, 
            std::vector<std::byte>& out_response_blob
        );
        ri::Response processMoveJoint(
            const rc::start::requests::deserialization::MoveJointRequest& move_joint_request,
            std::vector<std::byte>& out_response_blob
        );
        ri::Response processMoveLinear(
            const rc::start::requests::deserialization::MoveLinearRequest& move_linear_request,
            std::vector<std::byte>& out_response_blob
        );
        ri::Response processMoveArc(
            const rc::start::requests::deserialization::MoveArcRequest& move_arc_request,
            std::vector<std::byte>& out_response_blob
        );
        ri::Response processMoveTrajectory(
            const rc::start::requests::deserialization::MoveTrajectoryRequest& move_trajectory_request,
            std::vector<std::byte>& out_response_blob
        );

        void handleUpdates();
        std::tuple<rc::RobotStatus, const char*> readRobotStatus();
        bool readRobotPosition(
            rc::Pose& out_base_pose, 
            rc::Pose& out_flange_pose,
            rc::Pose& out_end_effector_pose
        );

        AergoConnector* base_;

        CbunLogger cbun_logger_;
        std::unique_ptr<rpc::RpcServer> rpc_server_;
        std::thread rpc_server_thread_;
        std::atomic<bool> rpc_server_running_{false};

        std::thread trajectory_thread_;
        std::atomic<bool> trajectory_stop_requested_{false};

        std::vector<std::byte> response_blob_buffer_;

        uint64_t next_action_id_{1};

        std::optional<uint64_t> current_move_action_id_{std::nullopt};
        int64_t current_move_start_time_us_{0};
        const int64_t min_move_duration_us_{100000}; // do not report move as finished before this time has elapsed

        struct {
            u_int16_t server_port;
        } activation_parameters_;

        uint32_t last_safety_flags_{0};
        uint32_t last_motion_flags_cleared_{0};
    };
}