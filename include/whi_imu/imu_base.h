/******************************************************************
imu base for abstract interface

Features:
- abstract imu operation interfaces
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
#include <string>
#include <memory>

class ImuBase
{
public:
	ImuBase() = delete;
	ImuBase(std::shared_ptr<ros::NodeHandle>& NodeHandle);
	virtual ~ImuBase() = default;

public:
	void setPublishParams(const std::string& FrameId, const std::string& DataTopic, const std::string& MagTopic, const std::string& TempTopic);
	virtual void read2Publish() = 0;
	virtual bool reset() = 0;

protected:
	std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	std::string frame_id_{ "imu_link" };
	std::string data_topic_{ "imu_data" };
	std::string mag_topic_{ "mag_data" };
	std::string temp_topic_{ "temp_data" };
	std::unique_ptr<ros::Publisher> pub_data_{ nullptr };
	std::unique_ptr<ros::Publisher> pub_mag_{ nullptr };
	std::unique_ptr<ros::Publisher> pub_temp_{ nullptr };
	bool reset_{ false };
};
