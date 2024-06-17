/******************************************************************
imu interface under ROS 1

Features:
- abstract imu interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_imu/whi_imu.h"
#include "whi_imu/imu_wit.h"

namespace whi_motion_interface
{
    const char* Imu::type_str[TYPE_SUM] = { "jy61p", "jy901" };

    Imu::Imu(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    Imu::~Imu()
    {
        if (srv_reset_)
        {
            srv_reset_->shutdown();
        }
    }

    void Imu::init()
    {
        // reset param
        bool resetAtInitial = false;
        node_handle_->param("reset_z", resetAtInitial, false);
        // drivers
        std::string frameId;
        std::string dataTopic;
        std::string magTopic;
        std::string tempTopic;
        node_handle_->param("frame_id", frameId, std::string("imu_link"));
        node_handle_->param("data_topic", dataTopic, std::string("imu_data"));
        node_handle_->param("mag_topic", magTopic, std::string("mag_data"));
        node_handle_->param("temp_topic", tempTopic, std::string("temp_data"));
        std::string module;
        std::string port;
        int baudrate = 0;
        int packLength = 0;
        std::shared_ptr<std::vector<int>> resetList = std::make_shared<std::vector<int>>();
        std::shared_ptr<std::vector<int>> unlockList = std::make_shared<std::vector<int>>();
        int instructionMinSpan = 5;
        bool withMag = false;
        bool withTemp = false;
        node_handle_->param("hardware_interface/module", module, std::string(type_str[WIT_JY61P]));
        node_handle_->param("hardware_interface/port", port, std::string("/dev/ttyUSB0"));
        node_handle_->param("hardware_interface/baudrate", baudrate, 9600);
        node_handle_->param("hardware_interface/pack_length", packLength, 11);
        node_handle_->getParam("hardware_interface/reset_yaw", *resetList);
        node_handle_->getParam("hardware_interface/unlock", *unlockList);
        node_handle_->param("hardware_interface/instruction_min_span", instructionMinSpan, 5);
        node_handle_->param("hardware_interface/with_magnetic", withMag, true);
        node_handle_->param("hardware_interface/with_temperature", withTemp, false);
        transform(module.begin(), module.end(), module.begin(), ::tolower);
        if (module == type_str[WIT_JY61P] || module == type_str[WIT_JY901] )
        {
            imu_inst_ = std::make_unique<ImuWit>(node_handle_,
                module, port, baudrate, packLength, resetList, unlockList, instructionMinSpan, withMag, withTemp);
        }
        else
        {
            imu_inst_ = std::make_unique<ImuWit>(node_handle_, module, port, baudrate, packLength, resetList);
        }
        bool debugYaw;
        node_handle_->param("debug_yaw", debugYaw, false);
        imu_inst_->setPublishParams(frameId, dataTopic, magTopic, tempTopic);
        imu_inst_->init(resetAtInitial);
		imu_inst_->debugYaw(debugYaw);

        // providing the reset service
        srv_reset_ = std::make_unique<ros::ServiceServer>(node_handle_->advertiseService("imu_reset",
            &Imu::onServiceReset, this));

        // spinner
        node_handle_->param("loop_hz", loop_hz_, 10.0);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq,
            std::bind(&Imu::update, this, std::placeholders::_1)));
    }

    void Imu::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        imu_inst_->read2Publish();
    }

    bool Imu::onServiceReset(std_srvs::Trigger::Request& Req, std_srvs::Trigger::Response& Res)
    {
        if (imu_inst_->reset())
        {
            Res.success = true;
            Res.message = "reset succeed";
        }
        else
        {
            Res.success = false;
            Res.message = "failed to reset";
        }

        return Res.success;
    }
}
