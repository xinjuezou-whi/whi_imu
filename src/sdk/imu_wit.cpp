/******************************************************************
imu driver instance for WIT brand

Features:
- imu operation logic for onboard hardware
- xxx

Prerequisites:
- git clone https://github.com/xinjuezou-whi/serial.git
  colcon build --symlink-install --packages-select serial

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_imu/imu_wit.h"

#include <tf2/LinearMath/Quaternion.h>

#include <cstring>
#include <unistd.h>
#include <iomanip>

const double ImuWit::CONSTANT = 32768.0;
const double ImuWit::CONSTANT_ACC = 16.0 * 9.8 / ImuWit::CONSTANT;
const double ImuWit::CONSTANT_GYRO = 2000.0 * WHI_PI / (180.0 * ImuWit::CONSTANT);
const double ImuWit::CONSTANT_ANGLE = WHI_PI / ImuWit::CONSTANT;
const double ImuWit::CONSTANT_QUATERNION = 1.0 / ImuWit::CONSTANT;
ImuWit::ImuWit(std::shared_ptr<rclcpp::Node>& NodeHandle, const std::string& Module,
	const std::string& SerPort, unsigned int Baudrate, unsigned int PackLength,
	const std::shared_ptr<std::vector<int>> ResetYaw, const std::shared_ptr<std::vector<int>> Unlock/* = nullptr*/, int InstructionMinSpan/* = 5*/,
	bool WithMagnetic/* = true*/, bool WithTemperature/* = false*/)
	: ImuBase(NodeHandle, WithMagnetic, WithTemperature), module_(Module)
	, serial_port_(SerPort), baudrate_(Baudrate), pack_length_(PackLength)
	, instruction_min_span_(1000 * InstructionMinSpan)
{
	// unlock and reset yaw commands
	if (ResetYaw)
	{
		for (auto it : *ResetYaw)
		{
			reset_yaw_.push_back((uint8_t)it);
		}
	}
	if (Unlock)
	{
		for (auto it : *Unlock)
		{
			unlock_.push_back((uint8_t)it);
		}
	}

#ifdef DEBUG
	std::cout << "acc const " << 1 / 32768.00 * 16 * 9.8 << " const " << CONSTANT_ACC << std::endl;
	std::cout << "gyro const " << 1 / 32768.00 * 2000 / 180 * WHI_PI << " const " << CONSTANT_GYRO << std::endl;
	std::cout << "angle const " << 1 / CONSTANT * WHI_PI << " const " << CONSTANT_ANGLE << std::endl;
	std::cout << "quat const " << 1 / CONSTANT << " const " << CONSTANT_QUATERNION << std::endl;
#endif
}

ImuWit::~ImuWit()
{
	if (serial_inst_)
	{
		serial_inst_->close();
	}
}

bool ImuWit::init(bool ResetAtInitial/* = false*/)
{
	reset_ = ResetAtInitial;
	// serial
	try
	{
		serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
		if (reset_)
		{
			reset();
		}

		return true;
	}
	catch (serial::IOException& e)
	{
		RCLCPP_FATAL(node_handle_->get_logger(),
			"\033[1;31mfailed to open serial %s\033[0m", serial_port_.c_str());

		return false;
	}
}

void ImuWit::read2Publish()
{
	if (serial_inst_)
	{
		size_t count = serial_inst_->available();
		if (count > 0)
		{
			unsigned char rbuff[count];
			size_t readNum = serial_inst_->read(rbuff, count);
			fetchData(rbuff, readNum);

			sensor_msgs::msg::Imu imuData;
			imuData.header.stamp = node_handle_->get_clock()->now();
			imuData.header.frame_id = frame_id_;

			imuData.linear_acceleration.x = acc_.x;
			imuData.linear_acceleration.y = acc_.y;
			imuData.linear_acceleration.z = acc_.z;
			imuData.linear_acceleration_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

			imuData.angular_velocity.x = gyro_.x;
			imuData.angular_velocity.y = gyro_.y;
			imuData.angular_velocity.z = gyro_.z;
			imuData.linear_acceleration_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

			// JY-61/61P, JY-901 have no quartnion output
			if (module_.find("61") != std::string::npos ||
				module_.find("901") != std::string::npos)
			{
				tf2::Quaternion convertQuaternion;
				convertQuaternion.setRPY(angle_.r, angle_.p, angle_.y);
				imuData.orientation.x = convertQuaternion.getX();
				imuData.orientation.y = convertQuaternion.getY();
				imuData.orientation.z = convertQuaternion.getZ();
				imuData.orientation.w = convertQuaternion.getW();
			}
			else
			{
				imuData.orientation.x = quaternion_.x;
				imuData.orientation.y = quaternion_.y;
				imuData.orientation.z = quaternion_.z;
				imuData.orientation.w = quaternion_.w;
			}
			imuData.orientation_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };
			pub_data_->publish(imuData);

			if (pub_mag_)
			{
				sensor_msgs::msg::MagneticField magData;
				magData.header.stamp = imuData.header.stamp;
				magData.header.frame_id = imuData.header.frame_id;

				magData.magnetic_field.x = magnetic_.x;
				magData.magnetic_field.y = magnetic_.y;
				magData.magnetic_field.z = magnetic_.z;
				magData.magnetic_field_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

				pub_mag_->publish(magData);
			}
			if (pub_temp_)
			{
				sensor_msgs::msg::Temperature tempData;
				tempData.header.stamp = imuData.header.stamp;;
				tempData.header.frame_id = imuData.header.frame_id;
				//tempData.temperature = (double)record.temperature;

				pub_temp_->publish(tempData);
			}
		}
	}
}

bool ImuWit::reset()
{
	// reset the yaw as 0
	if (!unlock_.empty() && serial_inst_)
	{
		serial_inst_->write(unlock_);
		usleep(instruction_min_span_);
	}
	if (!reset_yaw_.empty() && serial_inst_)
	{
		serial_inst_->write(reset_yaw_);
		usleep(instruction_min_span_);
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
			Array.push_back(Str.substr(pos, sepPos - pos));

			pos = sepPos + 1;
			sepPos = Str.find(Sep, pos);
		}
		if (pos < Str.length())
		{
			Array.push_back(Str.c_str() + pos);
		}
	}
}

void ImuWit::convert2Hex(std::vector<std::string>& Array, std::vector<uint8_t>& HexArray)
{
	for (const auto& it : Array)
	{
		std::stringstream converter;
		converter << std::hex << it;
		int byte = 0;
		converter >> byte;
		HexArray.push_back(uint8_t(byte & 0xFF));
	}
}

void ImuWit::fetchData(unsigned char* Data, size_t Length)
{
	unsigned char* head = Data;
	size_t rollingLen = Length;
	while (rollingLen >= pack_length_)
	{
		if (head[0] != 0x55)
		{
			if (++head < (Data + Length - pack_length_))
			{
				continue;
			}
			else
			{
				break;
			}
		}

		int16_t raw[4] = { 0, 0, 0, 0 };

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

			if (debug_yaw_)
			{
				std::cout << "yaw: " << std::fixed << std::setprecision(2) << angle_.y << std::endl;
			}
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

		rollingLen -= pack_length_;
		head += pack_length_;
	}
}
