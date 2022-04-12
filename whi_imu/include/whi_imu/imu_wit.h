/******************************************************************
imu driver instance for WIT brand

Features:
- imu operation logic for onboard hardware
- xxx

Prerequisites:
- sudo apt install ros-<ros distro>-serial

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-04-04: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "imu_base.h"

#include <serial/serial.h>
#include <vector>

class ImuWit : public ImuBase
{
public:
	ImuWit() = delete;
	ImuWit(std::shared_ptr<ros::NodeHandle>& NodeHandle, const std::string& Module,
		const std::string& SerPort, unsigned int Baudrate, unsigned int PackLength,
		const std::string& Unlock, const std::string& ResetYaw,
		bool WithMagnetic = true, bool WithTemperature = false);
	~ImuWit() override;

public:
	// override
	void read2Publish() override;
	bool reset() override;

protected:
	void extract2Array(const std::string& Str, std::vector<std::string>& Array, const char Sep = '*');
	void init(const std::string& Unlock, const std::string& ResetYaw, bool WithMagnetic, bool WithTemperature);
	void fetchData(unsigned char* Data, size_t Length);

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
	std::string serial_port_;
	unsigned int baudrate_{ 9600 };
	size_t pack_length_{ 11 };
	std::vector<uint8_t> unlock_;
	std::vector<uint8_t> reset_yaw_;
	std::unique_ptr<serial::Serial> serial_inst_{ nullptr };
	Time time_;
	Triple acc_;
	Triple gyro_;
	Angle angle_;
	Triple magnetic_;
	Quat quaternion_;

protected:
	static const double CONSTANT;
	static const double CONSTANT_ACC;
	static const double CONSTANT_GYRO;
	static const double CONSTANT_ANGLE;
	static const double CONSTANT_QUATERNION;
};
