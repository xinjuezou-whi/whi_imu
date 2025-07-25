/******************************************************************
node to handle imu sensors

Features:
- imu resouces setup logic
- publish message on /imu_data
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-04-04: Initial version
2025-07-20: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#include <iostream>
#include <signal.h>
#include <functional>

#include "whi_imu/whi_imu.h"

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI imu VERSION 02.10.2" << std::endl;
	std::cout << "Copyright © 2022-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    rclcpp::init(argc, argv);

    // create node
    const std::string nodeName("whi_imu");
	auto nodeHandle = std::make_shared<rclcpp::Node>(nodeName);

	/// node logic
	auto imu = std::make_unique<whi_imu::Imu>(nodeHandle);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		imu.reset();

		// all the default sigint handler does is call shutdown()
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(nodeHandle);
    executor->spin();  // blocking until shutdown
#else
    rclcpp::spin(nodeHandle);
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
