/******************************************************************
imu interface under ROS 2

Features:
- abstract imu interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-04-04: Initial version
2025-07-20: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>

#include "imu_base.h"
#include "whi_interfaces/msg/whi_state.hpp"

namespace whi_imu
{
	class Imu
	{
    public:
        enum Type { WIT_JY61P = 0, WIT_JY901, WIT_HWT6053_CAN, TYPE_SUM };
        static const char* type_str[TYPE_SUM];

    public:
        Imu(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~Imu();

    protected:
        void init();
        void update();
        bool onServiceReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> Response);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        rclcpp::TimerBase::SharedPtr non_realtime_loop_{ nullptr };
        std::unique_ptr<ImuBase> imu_inst_{ nullptr };
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_{ nullptr };
        rclcpp::Publisher<whi_interfaces::msg::WhiState>::SharedPtr pub_state_{ nullptr };
        whi_interfaces::msg::WhiState state_msg_;
	};
}
