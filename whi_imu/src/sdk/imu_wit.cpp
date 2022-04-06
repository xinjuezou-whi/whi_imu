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

******************************************************************/
#include "whi_imu/imu_wit.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <cstring>

#define WHI_PI (std::atan(1.0) * 4.0)

const double ImuWit::CONSTANT = 32768.0;
const double ImuWit::CONSTANT_ACC = 16.0 * 9.8 / ImuWit::CONSTANT;
const double ImuWit::CONSTANT_GYRO = 2000.0 * WHI_PI / (180.0 * ImuWit::CONSTANT);
const double ImuWit::CONSTANT_ANGLE = WHI_PI / ImuWit::CONSTANT;
const double ImuWit::CONSTANT_QUATERNION = 1.0 / ImuWit::CONSTANT;
ImuWit::ImuWit(std::shared_ptr<ros::NodeHandle>& NodeHandle, 
	const std::string& SerPort, unsigned int Baudrate, unsigned int PackLength,
	const std::string& Unlock, const std::string& ResetYaw,
	bool WithMagnetic/* = true*/, bool WithTemperature/* = false*/)
	: ImuBase(NodeHandle)
	, serial_port_(SerPort), baudrate_(Baudrate), pack_length_(PackLength)
{
	init(Unlock, ResetYaw, WithMagnetic, WithTemperature);

#ifdef DEBUG
	std::cout << "acc const " << 1 / 32768.00 * 16 * 9.8 << " const " << CONSTANT_ACC << std::endl;
	std::cout << "gyro const " << 1 / 32768.00 * 2000 / 180 * WHI_PI << " const " << CONSTANT_GYRO << std::endl;
	std::cout << "angle const " << 1 / CONSTANT * WHI_PI << " const " << CONSTANT_ANGLE << std::endl;
	std::cout << "quat const " << 1 / CONSTANT << " const " << CONSTANT_QUATERNION << std::endl;
#endif
}

ImuWit::~ImuWit()
{
	if (pub_data_)
	{
		pub_data_->shutdown();
	}
	if (pub_mag_)
	{
		pub_mag_->shutdown();
	}
	if (pub_mag_)
	{
		pub_mag_->shutdown();
	}
}

void ImuWit::read2Publish()
{
	static unsigned int seq = 0;

	if (serial_inst_)
	{
		size_t count = serial_inst_->available();
		if (count > 0)
		{
			unsigned char rbuff[count];
			size_t readNum = serial_inst_->read(rbuff, count);
			fetchData(rbuff, readNum);

			sensor_msgs::Imu imuData;
			imuData.header.stamp = ros::Time::now();
			imuData.header.frame_id = frame_id_;
			imuData.header.seq = seq;

			imuData.linear_acceleration.x = acc_.x;
			imuData.linear_acceleration.y = acc_.y;
			imuData.linear_acceleration.z = acc_.z;
			imuData.linear_acceleration_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

			imuData.angular_velocity.x = gyro_.x;
			imuData.angular_velocity.y = gyro_.y;
			imuData.angular_velocity.z = gyro_.z;
			imuData.linear_acceleration_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

			imuData.orientation.x = quaternion_.x;
			imuData.orientation.y = quaternion_.y;
			imuData.orientation.z = quaternion_.z;
			imuData.orientation.w = quaternion_.w;
			imuData.orientation_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

			pub_data_->publish(imuData);
			if (pub_mag_)
			{
				sensor_msgs::MagneticField magData;
				magData.header.stamp = imuData.header.stamp;
				magData.header.frame_id = imuData.header.frame_id;
				magData.header.seq = seq;

				magData.magnetic_field.x = magnetic_.x;
				magData.magnetic_field.y = magnetic_.y;
				magData.magnetic_field.z = magnetic_.z;
				magData.magnetic_field_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

				pub_mag_->publish(magData);
			}
			if (pub_temp_)
			{
				sensor_msgs::Temperature tempData;
				tempData.header.stamp = imuData.header.stamp;;
				tempData.header.frame_id = imuData.header.frame_id;
				tempData.header.seq = seq;
				//tempData.temperature = (double)record.temperature;

				pub_temp_->publish(tempData);
			}

			++seq;
		}
	}
}

bool ImuWit::reset()
{
	// reset the yaw as 0
	if (!unlock_.empty() && serial_inst_)
	{
		serial_inst_->write(unlock_);
		usleep(5000);
	}
	if (!reset_yaw_.empty() && serial_inst_)
	{
		serial_inst_->write(reset_yaw_);
		usleep(5000);
	}
	else
	{
		return false;
	}
	
	return true;
}

void ImuWit::extract2Array(const std::string& Str, std::vector<std::string>& Array, const char Sep/* = '*'*/)
{
	size_t sepPos = Str.find(Sep);
	if (sepPos == std::string::npos)
	{
		if (!Str.empty())
		{
			Array.push_back(Str);
		}
	}
	else
	{
		size_t pos = 0;
		while (sepPos != std::string::npos)
		{
			Array.push_back(Str.substr(pos, sepPos));

			pos = sepPos + 1;
			sepPos = Str.find(Sep, pos);
		}
		if (pos < Str.length())
		{
			Array.push_back(Str.c_str() + pos);
		}
	}
}

void ImuWit::init(const std::string& Unlock, const std::string& ResetYaw, bool WithMagnetic, bool WithTemperature)
{
	// reset commands
	std::vector<std::string> separated;
	extract2Array(Unlock, separated);
	for (const auto& it : separated)
	{
		unlock_.push_back((uint8_t)std::stoi(it));
	}
	separated.clear();
	extract2Array(ResetYaw, separated);
	for (const auto& it : separated)
	{
		reset_yaw_.push_back((uint8_t)std::stoi(it));
	}

	// publisher
	pub_data_ = std::make_unique<ros::Publisher>(node_handle_->advertise<sensor_msgs::Imu>("imu_data", 10));
	if (WithMagnetic)
	{
		pub_mag_ = std::make_unique<ros::Publisher>(node_handle_->advertise<sensor_msgs::MagneticField>("mag_data", 10));
	}
	if (WithTemperature)
	{
		pub_temp_ = std::make_unique<ros::Publisher>(node_handle_->advertise<sensor_msgs::Temperature>("temp_data", 10));
	}

	// serial
	try
	{
		serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
	}
	catch (serial::IOException& e)
	{
		ROS_FATAL_STREAM_NAMED("failed to open serial %s", serial_port_.c_str());
	}
}

void ImuWit::fetchData(unsigned char* Data, size_t Length)
{
	unsigned char* head = Data;
	while (Length >= pack_length_)
	{
		if (head[0] != 0x55)
		{
			head++;
			continue;
		}

		unsigned int raw[4] = { 0, 0, 0, 0 };

		switch (head[1])
		{
		case 0x50:
			memcpy(&time_, &head[2], 8);
			break;
		case 0x51:
			memcpy(&raw, &head[2], 8);
			acc_.x = raw[0] * CONSTANT_ACC;
			acc_.y = raw[1] * CONSTANT_ACC;
			acc_.z = raw[2] * CONSTANT_ACC;
			break;
		case 0x52:
			memcpy(&raw, &head[2], 8);
			gyro_.x = raw[0] * CONSTANT_GYRO;
			gyro_.y = raw[1] * CONSTANT_GYRO;
			gyro_.z = raw[2] * CONSTANT_GYRO;
			break;
		case 0x53:
			memcpy(&raw, &head[2], 8);
			angle_.r = raw[0] * CONSTANT_ANGLE;
			angle_.p = raw[1] * CONSTANT_ANGLE;
			angle_.y = raw[2] * CONSTANT_ANGLE;
			break;
		case 0x54:
			memcpy(&raw, &head[2], 8);
			magnetic_.x = raw[0];
			magnetic_.y = raw[1];
			magnetic_.z = raw[2];
			break;
			// case 0x55:	memcpy(&stcDStatus,&chrTemp[2],8);break;
			// case 0x56:	memcpy(&stcPress,&chrTemp[2],8);break;
			// case 0x57:	memcpy(&stcLonLat,&chrTemp[2],8);break;
			// case 0x58:	memcpy(&stcGPSV,&chrTemp[2],8);break;
		case 0x59:
			memcpy(&raw, &head[2], 8);
			quaternion_.w = raw[0] * CONSTANT_QUATERNION;
			quaternion_.x = raw[1] * CONSTANT_QUATERNION;
			quaternion_.y = raw[2] * CONSTANT_QUATERNION;
			quaternion_.z = raw[3] * CONSTANT_QUATERNION;
			break;
		}

		Length -= pack_length_;
		head += pack_length_;
	}
}
