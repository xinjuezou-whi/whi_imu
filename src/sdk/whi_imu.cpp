/******************************************************************
imu interface under ROS 2

Features:
- abstract imu interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_imu/whi_imu.h"
#include "whi_imu/imu_wit.h"
#include "whi_imu/imu_wit_usbcan.h"
#include "whi_imu/imu_wit_canbus.h"

namespace whi_imu
{
    const char* Imu::type_str[TYPE_SUM] = { "jy61p", "jy901" ,"hwt6053_can" };

    Imu::Imu(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    Imu::~Imu()
    {
        if (srv_reset_)
        {
            srv_reset_.reset();
        }
    }

    void Imu::init()
    {
        // reset param
        node_handle_->declare_parameter<bool>("reset_z", false);
        bool resetAtInitial = node_handle_->get_parameter("reset_z").as_bool();

        // drivers
        node_handle_->declare_parameter<std::string>("frame_id", std::string("imu_link"));
        auto frameId = node_handle_->get_parameter("frame_id").as_string();
        node_handle_->declare_parameter<std::string>("data_topic", std::string("imu_data"));
        auto dataTopic = node_handle_->get_parameter("data_topic").as_string();
        node_handle_->declare_parameter<std::string>("mag_topic", std::string("mag_data"));
        auto magTopic = node_handle_->get_parameter("mag_topic").as_string();
        node_handle_->declare_parameter<std::string>("temp_topic", std::string("temp_data"));
        auto tempTopic = node_handle_->get_parameter("temp_topic").as_string();

        node_handle_->declare_parameter<std::string>("hardware_interface.module", std::string(type_str[WIT_JY61P]));
        auto module = node_handle_->get_parameter("hardware_interface.module").as_string();
        transform(module.begin(), module.end(), module.begin(), ::tolower);

        node_handle_->declare_parameter<std::string>("hardware_interface.hardware_mode", std::string("usbcan"));
        auto hardwareMode = node_handle_->get_parameter("hardware_interface.hardware_mode").as_string();
        node_handle_->declare_parameter<std::string>("hardware_interface.port", std::string("/dev/ttyUSB0"));
        auto port = node_handle_->get_parameter("hardware_interface.port").as_string();
        node_handle_->declare_parameter<int>("hardware_interface.baudrate", 9600);
        int baudrate = node_handle_->get_parameter("hardware_interface.baudrate").as_int();
        node_handle_->declare_parameter<int>("hardware_interface.pack_length", 11);
        int packLength = node_handle_->get_parameter("hardware_interface.pack_length").as_int();

        node_handle_->declare_parameter<std::vector<int64_t>>("hardware_interface.reset_yaw", std::vector<int64_t>());
        auto reset = node_handle_->get_parameter("hardware_interface.reset_yaw").as_integer_array();
        node_handle_->declare_parameter<std::vector<int64_t>>("hardware_interface.unlock", std::vector<int64_t>());
        auto unlock = node_handle_->get_parameter("hardware_interface.unlock").as_integer_array();
        std::shared_ptr<std::vector<int>> resetList = std::make_shared<std::vector<int>>();
        for (const auto& it : reset)
        {
            resetList->push_back(int(it));
        }
        std::shared_ptr<std::vector<int>> unlockList = std::make_shared<std::vector<int>>();
        for (const auto& it : unlock)
        {
            unlockList->push_back(int(it));
        }
        
        node_handle_->declare_parameter<int>("hardware_interface.instruction_min_span", 5);
        int instructionMinSpan = node_handle_->get_parameter("hardware_interface.instruction_min_span").as_int();

        node_handle_->declare_parameter<bool>("hardware_interface.with_magnetic", true);
        bool withMag = node_handle_->get_parameter("hardware_interface.with_magnetic").as_bool();
        node_handle_->declare_parameter<bool>("hardware_interface.with_temperature", false);
        bool withTemp = node_handle_->get_parameter("hardware_interface.with_temperature").as_bool();

        if (module == type_str[WIT_JY61P] || module == type_str[WIT_JY901] )
        {
            imu_inst_ = std::make_unique<ImuWit>(node_handle_,
                module, port, baudrate, packLength, resetList, unlockList, instructionMinSpan, withMag, withTemp);
        }
        else if (module == type_str[WIT_HWT6053_CAN])
        {
            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".baudrate", 500);
            baudrate = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".baudrate").as_int();
            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".pack_length", 11);
            packLength = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".pack_length").as_int(); 
            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".device_addr", 0);
            int deviceAddr = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".device_addr").as_int(); 
            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".is_remote", false);
            bool isRemote = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".is_remote").as_bool();
            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".is_extended", false);
            bool isExtended = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".is_extended").as_bool();

            node_handle_->declare_parameter<std::vector<int64_t>>("hardware_interface." + hardwareMode + ".reset_yaw",
                std::vector<int64_t>());
            auto resetCan = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".reset_yaw").as_integer_array();
            node_handle_->declare_parameter<std::vector<int64_t>>("hardware_interface." + hardwareMode + ".unlock",
                std::vector<int64_t>());
            auto unlockCan = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".unlock").as_integer_array();
            std::shared_ptr<std::vector<int>> canresetList = std::make_shared<std::vector<int>>();
            for (const auto& it : resetCan)
            {
                canresetList->push_back(int(it));
            }
            std::shared_ptr<std::vector<int>> canunlockList = std::make_shared<std::vector<int>>();
            for (const auto& it : unlockCan)
            {
                canunlockList->push_back(int(it));
            }

            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".instruction_min_span", 5);
            instructionMinSpan = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".instruction_min_span").as_int();
            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".with_magnetic", true);
            withMag = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".with_magnetic").as_bool();
            node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".with_temperature", false);
            withTemp = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".with_temperature").as_bool();

            if (hardwareMode == "usbcan")
            {
                node_handle_->declare_parameter<int>("hardware_interface." + hardwareMode + ".bus_addr", 0);
                int busAddr = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".bus_addr").as_int();

                imu_inst_ = std::make_unique<ImuWitUsbcan>(node_handle_, module, busAddr, deviceAddr, baudrate, packLength, 
                    canresetList, canunlockList, instructionMinSpan, withMag, withTemp);
            }
            else if (hardwareMode == "canbus")
            {
                node_handle_->declare_parameter<std::string>("hardware_interface." + hardwareMode + ".bus_addr",
                    std::string("can0"));
                auto busAddr = node_handle_->get_parameter("hardware_interface." + hardwareMode + ".bus_addr").as_string();

                imu_inst_ = std::make_unique<ImuWitCanbus>(node_handle_, module, busAddr, deviceAddr, packLength,
                    canresetList, canunlockList, instructionMinSpan, withMag, withTemp);
            }
        }
        else
        {
            imu_inst_ = std::make_unique<ImuWit>(node_handle_, module, port, baudrate, packLength, resetList);
        }

        // state publisher
        pub_state_ = node_handle_->create_publisher<whi_interfaces::msg::WhiState>("whi_state", 10);
        state_msg_.hardware_id = "whi_imu";
        state_msg_.values.push_back(diagnostic_msgs::msg::KeyValue());

        node_handle_->declare_parameter<bool>("print_yaw", false);
        bool printYaw = node_handle_->get_parameter("print_yaw").as_bool();

        imu_inst_->setPublishParams(frameId, dataTopic, magTopic, tempTopic);
        imu_inst_->init(resetAtInitial);
		imu_inst_->printYaw(printYaw);

        // providing the reset service
        srv_reset_ = node_handle_->create_service<std_srvs::srv::Trigger>("imu_reset",
            std::bind(&Imu::onServiceReset, this, std::placeholders::_1, std::placeholders::_2));

        // spinner
        node_handle_->declare_parameter<double>("frequency", 10.0);
        double frequency = node_handle_->get_parameter("frequency").as_double();
        auto period = std::chrono::duration<double>(1.0 / frequency);
        non_realtime_loop_ = node_handle_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&Imu::update, this));
    }

    void Imu::update()
    {
        imu_inst_->read2Publish();

        auto current = node_handle_->get_clock()->now();
        static auto last = current;
        if ((current - last).seconds() > 0.2)
        {
            state_msg_.header.stamp = node_handle_->get_clock()->now();
            state_msg_.level = whi_interfaces::msg::WhiState::INFO;
            state_msg_.values.back().key = "state";
            state_msg_.values.back().value = "running";
        
            pub_state_->publish(state_msg_);

            last = current;
        }
    }

    bool Imu::onServiceReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> Response)
    {
        if (imu_inst_->reset())
        {
            Response->success = true;
            Response->message = "reset succeed";
        }
        else
        {
            Response->success = false;
            Response->message = "failed to reset";
        }

        return Response->success;
    }
}
