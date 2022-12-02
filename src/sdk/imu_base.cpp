/******************************************************************
imu base for abstract interface

Features:
- abstract imu operation interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_imu/imu_base.h"

ImuBase::ImuBase(std::shared_ptr<ros::NodeHandle>& NodeHandle)
	: node_handle_(NodeHandle)
{
	node_handle_->param("reset_z", reset_, false);
}

void ImuBase::setPublishParams(const std::string& FrameId, const std::string& DataTopic, const std::string& MagTopic, const std::string& TempTopic)
{
	frame_id_.assign(FrameId);
	data_topic_.assign(DataTopic);
	mag_topic_.assign(MagTopic);
	temp_topic_.assign(TempTopic);
}
