#include "whi_imu/imu_wit_canbus.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <cstring>
#include <angles/angles.h>
//#define DEBUG 1
#define WHI_PI (std::atan(1.0) * 4.0)

const double ImuWitCanbus::CONSTANT = 32768.0;
const double ImuWitCanbus::CONSTANT_ACC = 16.0 * 9.8 / ImuWitCanbus::CONSTANT;
const double ImuWitCanbus::CONSTANT_GYRO = 2000.0 * WHI_PI / (180.0 * ImuWitCanbus::CONSTANT);
const double ImuWitCanbus::CONSTANT_ANGLE = 0.001;
const double ImuWitCanbus::CONSTANT_QUATERNION = 1.0 / ImuWitCanbus::CONSTANT;

ImuWitCanbus::ImuWitCanbus(std::shared_ptr<ros::NodeHandle>& NodeHandle, const std::string& Module, 
	const std::string& BusAddr, uint16_t DeviceAddr, unsigned int PackLength,
	const std::shared_ptr<std::vector<int>> ResetYaw, const std::shared_ptr<std::vector<int>> Unlock,  int InstructionMinSpan,
		bool WithMagnetic, bool WithTemperature)
	: ImuBase(NodeHandle), bus_addr_(BusAddr), device_addr_(DeviceAddr)
	, instruction_min_span_(1000 * InstructionMinSpan), with_magnetic_(WithMagnetic), with_temperature_(WithTemperature)	
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

    bus_ = std::make_shared<CanBus>(bus_addr_);
	if (bus_)
	{
		if (bus_->open())
        {
			std::cout << "canbus open success !" << std::endl;
		}
	}

	// spawn the read thread
	th_read_ = std::thread(std::bind(&ImuWitCanbus::threadReadCan, this));
#ifdef DEBUG
	std::cout << "acc const " << 1 / 32768.00 * 16 * 9.8 << " const " << CONSTANT_ACC << std::endl;
	std::cout << "gyro const " << 1 / 32768.00 * 2000 / 180 * WHI_PI << " const " << CONSTANT_GYRO << std::endl;
	std::cout << "angle const " << 1 / CONSTANT * WHI_PI << " const " << CONSTANT_ANGLE << std::endl;
	std::cout << "quat const " << 1 / CONSTANT << " const " << CONSTANT_QUATERNION << std::endl;
#endif
}

ImuWitCanbus::~ImuWitCanbus()
{
	terminated_.store(true);
	if (th_read_.joinable())
	{
		th_read_.join();
	}

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

    if (bus_ && bus_->isOpen())
	{
		bus_->close();
	}
}

void ImuWitCanbus::setPublishParams(const std::string& FrameId, const std::string& DataTopic,
	const std::string& MagTopic, const std::string& TempTopic)
{
	frame_id_.assign(FrameId);
	data_topic_.assign(DataTopic);
	mag_topic_.assign(MagTopic);
	temp_topic_.assign(TempTopic);

	reconfigPub();
}

void ImuWitCanbus::reconfigPub()
{
	// publisher
	pub_data_ = std::make_unique<ros::Publisher>(node_handle_->advertise<sensor_msgs::Imu>(data_topic_, 10));
	if (with_magnetic_)
	{
		pub_mag_ = std::make_unique<ros::Publisher>(node_handle_->advertise<sensor_msgs::MagneticField>(mag_topic_, 10));
	}
	if (with_temperature_)
	{
		pub_temp_ = std::make_unique<ros::Publisher>(node_handle_->advertise<sensor_msgs::Temperature>(temp_topic_, 10));
	}
}

bool ImuWitCanbus::init(bool ResetAtInitial/* = false*/)
{
	if (ResetAtInitial)
	{
		reset();
	}

	return bus_ ? true : false;
}


void ImuWitCanbus::read2Publish()
{
	static unsigned int seq = 0;

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

	//convertQuaternion.setRPY(angles::from_degrees(angle_.r), angles::from_degrees(angle_.p), angles::from_degrees(angle_.y));
	imuData.orientation.x = convertQuaternion_.getX();
	imuData.orientation.y = convertQuaternion_.getY();
	imuData.orientation.z = convertQuaternion_.getZ();
	imuData.orientation.w = convertQuaternion_.getW();
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
		tempData.header.stamp = imuData.header.stamp;
		tempData.header.frame_id = imuData.header.frame_id;
		tempData.header.seq = seq;
		//tempData.temperature = (double)record.temperature;

		pub_temp_->publish(tempData);
	}

	++seq;
}

bool ImuWitCanbus::reset()
{
	// reset the yaw as 0
	if (!unlock_.empty() && bus_)
	{
		unsigned int ID = device_addr_;
		size_t Len = unlock_.size();
		bus_->write(ID, Len, unlock_.data());
		usleep(instruction_min_span_);
	}
	if (!reset_yaw_.empty() && bus_)
	{
		unsigned int ID = device_addr_;
		size_t Len = reset_yaw_.size();		
		bus_->write(ID, Len,reset_yaw_.data());
		usleep(instruction_min_span_);
	}
	else
	{
		return false;
	}
		
	return true;
}

void ImuWitCanbus::threadReadCan()
{
	static unsigned int seq = 0;

	const int DATA_LEN = 8;
	const int DATA_LEN_MIN = 4;
    unsigned int id = 0;
    std::array<uint8_t, 8> read;

	while (!terminated_.load())
	{
		if (bus_ && bus_->eventTriggered(id, read.data()) >= DATA_LEN_MIN)
		{
			switch (read[1])
			{
				case 0x50:
					//memcpy(&time_, &head[2], 8);
					break;
				case 0x51:
					acc_.x = short((short)read[3] << 8 | read[2] ) * CONSTANT_ACC;
					acc_.y = short((short)read[5] << 8 | read[4] ) * CONSTANT_ACC;
					acc_.z = short((short)read[7] << 8 | read[6] ) * CONSTANT_ACC;
					break;
				case 0x52:
					gyro_.x = short((short)read[3] << 8 | read[2] ) * CONSTANT_GYRO;
					gyro_.y = short((short)read[5] << 8 | read[4] ) * CONSTANT_GYRO;
					gyro_.z = short((short)read[7] << 8 | read[6] ) * CONSTANT_GYRO;
					break;
				case 0x53:
					static int anglepack = 0x000;
					if (read[2] == 0x01)
					{
						angle_.r = float((int)read[7] << 24 | (int)read[6] << 16 | (int)read[5] << 8 | (int)read[4]) * CONSTANT_ANGLE;
						anglepack |= 0x001;
					}
					else if (read[2] == 0x02)
					{
						angle_.p = float((int)read[7] << 24 | (int)read[6] << 16 | (int)read[5] << 8 | (int)read[4]) * CONSTANT_ANGLE;
						anglepack |= 0x010;
					}
					else if (read[2] == 0x03)
					{
						angle_.y = float((int)read[7] << 24 | (int)read[6] << 16 | (int)read[5] << 8 | (int)read[4]) * CONSTANT_ANGLE;
						anglepack |= 0x100;
						if (debug_yaw_)
						{
							printf("yaw %.2f\n", angle_.y);
						}
					}
					if (anglepack == 0x111)
					{
						// update quternion
						convertQuaternion_.setRPY(angles::from_degrees(angle_.r), angles::from_degrees(angle_.p), angles::from_degrees(angle_.y));
						anglepack = 0x000;
					}
					break;
				case 0x54:
					magnetic_.x = short((short)read[3] << 8 | read[2] );
					magnetic_.y = short((short)read[5] << 8 | read[4] );
					magnetic_.z = short((short)read[7] << 8 | read[6] );
					break;
					// case 0x55:	memcpy(&stcDStatus,&chrTemp[2],8);break;
					// case 0x56:	memcpy(&stcPress,&chrTemp[2],8);break;
					// case 0x57:	memcpy(&stcLonLat,&chrTemp[2],8);break;
					// case 0x58:	memcpy(&stcGPSV,&chrTemp[2],8);break;
			}

#ifdef DEBUG
			std::cout << "imu raw " << std::endl;
			for (const auto& it : read)
			{
				std::cout << std::hex << int(it) << ",";
			}
			//std::cout << std::dec << value << std::endl;
			std::cout << std::endl;
#endif

		}
	}
}