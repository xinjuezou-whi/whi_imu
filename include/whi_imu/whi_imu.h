/******************************************************************
imu interface under ROS 1

Features:
- abstract imu interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-04-04: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <memory>

#include "imu_base.h"

namespace whi_motion_interface
{
	class Imu
	{
    public:
        enum Type { WIT_JY61P = 0, TYPE_SUM };
        static const char* type_str[TYPE_SUM];

    public:
        Imu(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~Imu();

    protected:
        void init();
        void update(const ros::TimerEvent & Event);
        bool onServiceReset(std_srvs::Trigger::Request& Req, std_srvs::Trigger::Response& Res);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_hz_{ 10.0 };
        std::unique_ptr<ImuBase> imu_inst_{ nullptr };
        std::unique_ptr<ros::ServiceServer> srv_reset_{ nullptr };
	};
}
