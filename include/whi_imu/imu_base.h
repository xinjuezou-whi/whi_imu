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
2025-07-20: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <string>
#include <memory>

#define WHI_PI (std::atan(1.0) * 4.0)

class ImuBase
{
public:
	ImuBase() = delete;
	ImuBase(std::shared_ptr<rclcpp::Node>& NodeHandle, bool WithMagnetic = true, bool WithTemperature = false)
		: node_handle_(NodeHandle), with_magnetic_(WithMagnetic), with_temperature_(WithTemperature) {};
	virtual ~ImuBase()
	{
		if (pub_data_)
		{
			pub_data_.reset();
		}
		if (pub_mag_)
		{
			pub_mag_.reset();
		}
		if (pub_temp_)
		{
			pub_temp_.reset();
		}
	};

public:
	void setPublishParams(const std::string& FrameId, const std::string& DataTopic,
		const std::string& MagTopic, const std::string& TempTopic)
	{
		frame_id_.assign(FrameId);
		data_topic_.assign(DataTopic);
		mag_topic_.assign(MagTopic);
		temp_topic_.assign(TempTopic);

		reconfigPub();
	};
	void reconfigPub()
	{
		// publisher
		pub_data_ = node_handle_->create_publisher<sensor_msgs::msg::Imu>(data_topic_, 10);
		if (with_magnetic_)
		{
			pub_mag_ = node_handle_->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_, 10);
		}
		if (with_temperature_)
		{
			pub_temp_ = node_handle_->create_publisher<sensor_msgs::msg::Temperature>(temp_topic_, 10);
		}
	};
	virtual bool init(bool ResetAtInitial = false) = 0;
	void debugYaw(bool Flag) { debug_yaw_ = Flag; };
	virtual void read2Publish() = 0;
	virtual bool reset() = 0;

protected:
	std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
	std::string frame_id_{ "imu_link" };
	std::string data_topic_{ "imu_data" };
	std::string mag_topic_{ "mag_data" };
	std::string temp_topic_{ "temp_data" };
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_data_{ nullptr };
	rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_{ nullptr };
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_{ nullptr };
	bool with_magnetic_{ true };
	bool with_temperature_{ true };
	bool reset_{ false };
	bool debug_yaw_{ false };
};
