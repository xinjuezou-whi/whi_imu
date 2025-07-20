/******************************************************************
imu driver instance for WIT brand

Features:
- imu operation logic for usbcan hardware
- xxx

Prerequisites:
-

Written by Yue Zhou, sevendull@163.com
           Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-08-09: Initial version
2025-07-20: Migrate from ROS 1 by Xinjue
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "imu_base.h"
#include "usbcan.h"

#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <thread>
#include <mutex>

class ImuWitUsbcan : public ImuBase
{
public:
	ImuWitUsbcan() = delete;
	ImuWitUsbcan(std::shared_ptr<rclcpp::Node>& NodeHandle, const std::string& Module, 
		uint8_t BusAddr, uint16_t DeviceAddr, int Baudrate, unsigned int PackLength,
		const std::shared_ptr<std::vector<int>> ResetYaw, const std::shared_ptr<std::vector<int>> Unlock = nullptr,
		int InstructionMinSpan = 5, bool WithMagnetic = true, bool WithTemperature = false);
	~ImuWitUsbcan() override;

public:
	// override
	bool init(bool ResetAtInitial = false) override;
	void read2Publish() override;
	bool reset() override;

protected:
	void threadReadCan();

protected:
	struct Time
	{
		int year;
		int month;
		int day;
		int hour;
		int min;
		int sec;
		time_t utc;
	};

	struct Triple
	{
		double x;
		double y;
		double z;
	};

	struct Angle
	{
		double r;
		double p;
		double y;
	};

	struct Quat
	{
		double x;
		double y;
		double z;
		double w;
	};

protected:
	std::string module_;
	std::string can_port_;
	std::uint16_t device_addr_{ 0x01 };
	size_t pack_length_{ 11 };
	std::vector<uint8_t> unlock_;
	std::vector<uint8_t> reset_yaw_;
	int instruction_min_span_{ 5 };
	Time time_;
	Triple acc_;
	Triple gyro_;
	Angle angle_;
	Triple magnetic_;
	Quat quaternion_;
	std::thread th_read_;
	std::atomic_bool terminated_{ false };
	uint8_t bus_addr_;
	std::shared_ptr<UsbCan> usbcan_;
	tf2::Quaternion convertQuaternion_;

protected:
	static const double CONSTANT;
	static const double CONSTANT_ACC;
	static const double CONSTANT_GYRO;
	static const double CONSTANT_ANGLE;
	static const double CONSTANT_QUATERNION;
};
