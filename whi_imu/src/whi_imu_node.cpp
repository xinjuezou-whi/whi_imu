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
2022-xx-xx: xxx
******************************************************************/
#include <iostream>

#include "whi_imu/whi_imu.h"

#define ASYNC 1

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI imu VERSION 00.01" << std::endl;
	std::cout << "Copyright © 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
	ros::init(argc, argv, "whi_imu");
	auto nodeHandle = std::make_shared<ros::NodeHandle>();

	/// node logic
	auto imu = std::make_unique<whi_motion_interface::Imu>(nodeHandle);

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
#else
	ros::MultiThreadedSpinner spinner(0);
	spinner.spin();
#endif

	std::cout << "whi_motion_interface exited" << std::endl;

	return 0;
}
