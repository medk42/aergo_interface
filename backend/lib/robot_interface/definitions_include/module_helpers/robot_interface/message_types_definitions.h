#pragma once

#include "module_common/module_interface_.h"

namespace aergo::module::helpers::robot_interface
{
    constexpr aergo::module::communication_channel::Consumer robot_interface_request_consumer = {
        .count_ = aergo::module::communication_channel::Consumer::Count::SINGLE,
        .channel_type_identifier_ = "robot_interface_req/v1:struct{req_type:uint8,feature:uint64,action_id:uint64}[+blob{data}]",
        .display_name_ = "Robot Control Request",
        .display_description_ = "Request channel for robot control.",
        .prioritized_ = true,
        .message_queue_capacity_ = 16
    };

    constexpr aergo::module::communication_channel::Producer robot_interface_response_producer = {
        .channel_type_identifier_ = "robot_interface_resp/v1:struct{resp_type:uint8,finished:bool,action_id:uint64}[+blob{data}]",
        .display_name_ = "Robot Control Response",
        .display_description_ = "Response channel for robot control actions.",
        .prioritized_ = true,
        .message_queue_capacity_ = 16
    };

    constexpr aergo::module::communication_channel::Producer robot_interface_status_producer = {
        .channel_type_identifier_ = "robot_interface_status/v1:struct{feature:uint64}[+blob{data}]",
        .display_name_ = "Robot Status Message",
        .display_description_ = "Asynchronous status messages from robot.",
        .prioritized_ = false,
        .message_queue_capacity_ = 16
    };

    constexpr aergo::module::communication_channel::Producer robot_interface_finished_producer = {
        .channel_type_identifier_ = "robot_interface_finished/v1:struct{action_id:uint64}[+blob{data}]",
        .display_name_ = "Robot Action Finished Message",
        .display_description_ = "Asynchronous finished messages for long-running robot actions.",
        .prioritized_ = true,
        .message_queue_capacity_ = 16
    };
}